package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.params.provider.Arguments.arguments;

import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.CsvFileSource;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;
import org.junit.jupiter.params.provider.MethodSource;
import org.junit.jupiter.params.provider.ValueSource;

class BrakeControllerTest {

    private static final double DELTA = 1e-9;

    private BrakeController controller;
    private GearBox.Gear gear;

    // Fresh fixture for every test: no shared state between tests.
    @BeforeEach
    void setUp() {
        controller = new BrakeController();
        gear = GearBox.Gear.D;
    }

    // ------------------------------------------------------------------
    // apply: input validation
    // ------------------------------------------------------------------

    // @ValueSource -> judged with assertThrows
    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testApplyRejectsInvalidPressure(double invalidPressure) {
        assertThrows(IllegalArgumentException.class, () -> controller.apply(invalidPressure, 50.0, gear));
    }

    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testApplyRejectsInvalidSpeed(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> controller.apply(50.0, invalidSpeed, gear));
    }

    @Test
    void testApplyRejectsParkGear() {
        assertThrows(IllegalStateException.class, () -> controller.apply(50.0, 10.0, GearBox.Gear.P));
    }

    @Test
    void testApplyRejectsNullGear() {
        assertThrows(NullPointerException.class, () -> controller.apply(50.0, 10.0, null));
    }

    // ------------------------------------------------------------------
    // apply: gear handling and non-ABS pressure
    // ------------------------------------------------------------------

    // @EnumSource (every gear except P) -> judged with assertEquals on the returned pressure
    @ParameterizedTest
    @EnumSource(value = GearBox.Gear.class, mode = EnumSource.Mode.EXCLUDE, names = "P")
    void testApplyAllowedInEveryNonParkGear(GearBox.Gear nonParkGear) {
        assertEquals(40.0, controller.apply(40.0, 10.0, nonParkGear), DELTA);
    }

    // @CsvSource -> judged with assertEquals + delta
    @ParameterizedTest
    @CsvSource({
            "0.0,   50.0,  0.0",    // zero request
            "50.0,  50.0,  50.0",   // plain pass-through
            "80.0,  100.0, 80.0",   // pressure exactly at ABS threshold: no ABS
            "100.0, 30.0,  100.0",  // speed exactly at ABS threshold: no ABS
            "150.0, 30.0,  100.0"   // over-max request is capped at 100
    })
    void testApplyWithoutAbsReturnsBoundedPressure(double requested, double speedKmh, double expected) {
        assertEquals(expected, controller.apply(requested, speedKmh, gear), DELTA);
    }

    // ------------------------------------------------------------------
    // apply: ABS
    // ------------------------------------------------------------------

    // @CsvFileSource -> judged with assertTrue / assertFalse (first pulse is 90, never a plain result here)
    @ParameterizedTest
    @CsvFileSource(resources = "/abs_activation.csv", numLinesToSkip = 1)
    void testAbsActivationThresholds(double requested, double speedKmh, boolean expectedAbs) {
        boolean absActive = controller.apply(requested, speedKmh, gear) == 90.0;

        if (expectedAbs) {
            assertTrue(absActive);
        } else {
            assertFalse(absActive);
        }
    }

    // @MethodSource -> judged with assertArrayEquals over the whole output sequence
    @ParameterizedTest
    @MethodSource("absRequestSequences")
    void testAbsPulseSequence(double[][] requests, double[] expectedPressures) {
        double[] actual = new double[requests.length];
        for (int i = 0; i < requests.length; i++) {
            actual[i] = controller.apply(requests[i][0], requests[i][1], gear);
        }

        assertArrayEquals(expectedPressures, actual, DELTA);
    }

    static Stream<Arguments> absRequestSequences() {
        // each request is { pressure, speedKmh }
        return Stream.of(
                // pulses alternate high/low
                arguments(new double[][] { { 95, 100 }, { 95, 100 }, { 95, 100 } },
                        new double[] { 90.0, 60.0, 90.0 }),
                // a non-ABS request resets the cycle to "high"
                arguments(new double[][] { { 95, 100 }, { 50, 100 }, { 95, 100 } },
                        new double[] { 90.0, 50.0, 90.0 }),
                // reset also happens when pressure drops while in the "low" phase
                arguments(new double[][] { { 95, 100 }, { 95, 100 }, { 70, 100 }, { 95, 100 } },
                        new double[] { 90.0, 60.0, 70.0, 90.0 }));
    }

    // ------------------------------------------------------------------
    // emergencyBrake
    // ------------------------------------------------------------------

    @Test
    void testEmergencyBrakeRejectsParkGear() {
        assertThrows(IllegalStateException.class, () -> controller.emergencyBrake(GearBox.Gear.P));
    }

    @Test
    void testEmergencyBrakeRejectsNullGear() {
        assertThrows(NullPointerException.class, () -> controller.emergencyBrake(null));
    }

    @Test
    void testEmergencyBrakeSetsFullPressureAndWarning() {
        controller.emergencyBrake(gear);

        assertAll(
                () -> assertEquals(100.0, controller.getPressure(), DELTA),
                () -> assertTrue(controller.isWarningActive()));
    }

    @Test
    void testEmergencyBrakeResetsAbsPulseCycle() {
        controller.apply(95.0, 100.0, gear); // 90, next pulse would be low
        controller.emergencyBrake(gear);

        assertEquals(90.0, controller.apply(95.0, 100.0, gear), DELTA);
    }

    // ------------------------------------------------------------------
    // warning flag
    // ------------------------------------------------------------------

    @Test
    void testApplyDoesNotActivateWarning() {
        controller.apply(95.0, 100.0, gear);

        assertFalse(controller.isWarningActive());
    }

    @Test
    void testWarningStaysActiveAfterLaterApply() {
        controller.emergencyBrake(gear);
        controller.apply(20.0, 10.0, gear);

        assertTrue(controller.isWarningActive());
    }
}
