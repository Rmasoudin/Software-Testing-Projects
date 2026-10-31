package autopilot.core;

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

class SteeringControllerTest {

    private static final double DELTA = 1e-9;

    private SteeringController controller;

    // Fresh controller (angle 0, target 0) for every test: no shared state.
    @BeforeEach
    void setUp() {
        controller = new SteeringController();
    }

    // Brings the wheel to the given angle (|angle| <= 450) at standstill, so the
    // controller is in a known state before the behaviour under test is exercised.
    private void driveTo(double angleDegrees) {
        for (int i = 0; i < 20 && controller.getCurrentAngleDegrees() != angleDegrees; i++) {
            controller.update(angleDegrees, 0.0);
        }
    }

    // Used by the @EnumSource test: which side of centre the wheel starts on.
    private enum Side {
        LEFT(-1.0), RIGHT(1.0);

        private final double sign;

        Side(double sign) {
            this.sign = sign;
        }
    }

    // ------------------------------------------------------------------
    // update: input validation
    // ------------------------------------------------------------------

    // @ValueSource -> judged with assertThrows (negative angles are valid, so only non-finite)
    @ParameterizedTest
    @ValueSource(doubles = { Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testUpdateRejectsNonFiniteAngle(double invalidAngle) {
        assertThrows(IllegalArgumentException.class, () -> controller.update(invalidAngle, 50.0));
    }

    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testUpdateRejectsInvalidSpeed(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> controller.update(10.0, invalidSpeed));
    }

    @Test
    void testUpdateRejectionLeavesStateUnchanged() {
        controller.update(100.0, 0.0); // current 60, target 100

        assertThrows(IllegalArgumentException.class, () -> controller.update(200.0, -1.0));

        assertEquals(100.0, controller.getTargetAngleDegrees(), DELTA);
        assertEquals(60.0, controller.getCurrentAngleDegrees(), DELTA);
    }

    // ------------------------------------------------------------------
    // update: target clamping by speed
    // ------------------------------------------------------------------

    // @CsvSource -> judged with assertEquals + delta on the clamped target
    @ParameterizedTest
    @CsvSource({
            "0.0,    0.0,    0.0",     // centre request
            "-30.0,  200.0,  -30.0",   // negative angle within the high-speed limit
            "450.0,  0.0,    450.0",   // exactly at the physical limit
            "500.0,  50.0,   450.0",   // above physical limit
            "-500.0, 50.0,   -450.0",  // below negative physical limit
            "200.0,  120.0,  200.0",   // speed exactly at threshold: still physical limit
            "200.0,  120.01, 90.0",    // just above threshold: high-speed limit
            "-200.0, 120.01, -90.0",   // same, negative side
            "90.0,   200.0,  90.0"     // exactly at the high-speed limit
    })
    void testUpdateClampsTargetToSpeedLimit(double requested, double speedKmh, double expectedTarget) {
        controller.update(requested, speedKmh);

        assertEquals(expectedTarget, controller.getTargetAngleDegrees(), DELTA);
    }

    // ------------------------------------------------------------------
    // update: rate limiting (60 degrees per tick)
    // ------------------------------------------------------------------

    // @MethodSource -> judged with assertArrayEquals over the whole sequence of ticks
    @ParameterizedTest
    @MethodSource("updateSequences")
    void testUpdateRateLimitsAngleChange(double[] requests, double speedKmh, double[] expectedAngles) {
        double[] actual = new double[requests.length];
        for (int i = 0; i < requests.length; i++) {
            actual[i] = controller.update(requests[i], speedKmh);
        }

        assertArrayEquals(expectedAngles, actual, DELTA);
    }

    static Stream<Arguments> updateSequences() {
        return Stream.of(
                // ramp up, last tick partial, then stays
                arguments(new double[] { 200, 200, 200, 200, 200 }, 0.0,
                        new double[] { 60, 120, 180, 200, 200 }),
                // ramp down (negative direction)
                arguments(new double[] { -200, -200, -200, -200 }, 0.0,
                        new double[] { -60, -120, -180, -200 }),
                // difference of exactly one step is reached in one tick
                arguments(new double[] { 60 }, 0.0, new double[] { 60 }),
                // difference just over one step needs two ticks
                arguments(new double[] { 60.01, 60.01 }, 0.0, new double[] { 60, 60.01 }),
                // direction reversal mid-ramp
                arguments(new double[] { 200, -200, -200 }, 0.0, new double[] { 60, 0, -60 }),
                // clamp to 450 combined with ramping
                arguments(new double[] { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 }, 0.0,
                        new double[] { 60, 120, 180, 240, 300, 360, 420, 450 }),
                // high speed: target clamped to 90, ramp 60 then 30
                arguments(new double[] { 300, 300, 300 }, 150.0, new double[] { 60, 90, 90 }));
    }

    // ------------------------------------------------------------------
    // selfCenter
    // ------------------------------------------------------------------

    // @CsvFileSource -> judged with assertTrue / assertFalse (centred after one tick or not)
    @ParameterizedTest
    @CsvFileSource(resources = "/self_center.csv", numLinesToSkip = 1)
    void testSelfCenterReachesCentreWithinOneStep(double startAngle, boolean expectedCentered) {
        driveTo(startAngle);

        boolean centered = controller.selfCenter() == 0.0;

        if (expectedCentered) {
            assertTrue(centered);
        } else {
            assertFalse(centered);
        }
    }

    // @EnumSource -> judged with assertEquals on the returned angle and on the reset target
    @ParameterizedTest
    @EnumSource(Side.class)
    void testSelfCenterMovesTowardZeroFromEitherSide(Side side) {
        driveTo(side.sign * 200.0);

        double result = controller.selfCenter();

        assertEquals(side.sign * 140.0, result, DELTA);
        assertEquals(0.0, controller.getTargetAngleDegrees(), DELTA);
    }
}
