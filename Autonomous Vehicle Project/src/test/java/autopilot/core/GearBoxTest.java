package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertAll;
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

import autopilot.core.GearBox.Gear;
import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;

class GearBoxTest {

    // Hand-written test double: reports a configurable speed and counts how often it is read.
    // JUnit creates a new test instance per test, so this state is never shared between tests.
    private double sensorSpeedKmh;
    private int sensorReadCount;
    private final SpeedSensor sensor = new SpeedSensor() {
        @Override
        public double getSpeedKmh() {
            sensorReadCount++;
            return sensorSpeedKmh;
        }
    };
    private GearBox gearBox;

    // Fresh gearbox (starts in P) for every test.
    @BeforeEach
    void setUp() {
        gearBox = new GearBox(sensor);
    }

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------

    private void stubSpeed(double speedKmh) {
        sensorSpeedKmh = speedKmh;
    }

    // Brings the gearbox to the given gear while stationary, then forgets the setup calls.
    private void moveTo(Gear target) {
        stubSpeed(0.0);
        switch (target) {
            case P -> { }
            case R -> gearBox.changeTo(Gear.R);
            case N -> {
                gearBox.changeTo(Gear.R);
                gearBox.changeTo(Gear.N);
            }
            case D -> {
                gearBox.changeTo(Gear.R);
                gearBox.changeTo(Gear.N);
                gearBox.changeTo(Gear.D);
            }
        }
        sensorReadCount = 0;
    }

    private boolean changeSucceeds(Gear target) {
        try {
            gearBox.changeTo(target);
            return true;
        } catch (IllegalGearChangeException e) {
            return false;
        }
    }

    // ------------------------------------------------------------------
    // changeTo: argument handling
    // ------------------------------------------------------------------

    @Test
    void testChangeToRejectsNullGear() {
        assertThrows(NullPointerException.class, () -> gearBox.changeTo(null));
    }

    // @EnumSource (all gears) -> judged with assertEquals on the sensor read count + resulting gear
    @ParameterizedTest
    @EnumSource(Gear.class)
    void testChangeToSameGearIsNoOpWithoutReadingSensor(Gear gear) {
        moveTo(gear);

        gearBox.changeTo(gear);

        assertEquals(0, sensorReadCount);
        assertEquals(gear, gearBox.getCurrentGear());
    }

    // @ValueSource -> judged with assertThrows (gear must also stay unchanged)
    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testChangeToRejectsInvalidSensorReading(double invalidSpeed) {
        stubSpeed(invalidSpeed);

        assertThrows(IllegalArgumentException.class, () -> gearBox.changeTo(Gear.R));
        assertEquals(Gear.P, gearBox.getCurrentGear());
    }

    // ------------------------------------------------------------------
    // changeTo: speed-dependent transitions (P->R, R->D, D->R)
    // ------------------------------------------------------------------

    // @CsvFileSource -> judged with assertTrue / assertFalse (allowed or rejected)
    @ParameterizedTest
    @CsvFileSource(resources = "/gear_speed_rules.csv", numLinesToSkip = 1)
    void testChangeToSpeedLimitedTransitions(Gear from, Gear to, double speedKmh, boolean expectedAllowed) {
        moveTo(from);
        stubSpeed(speedKmh);

        boolean allowed = changeSucceeds(to);

        if (expectedAllowed) {
            assertTrue(allowed);
        } else {
            assertFalse(allowed);
        }
    }

    // ------------------------------------------------------------------
    // changeTo: speed-independent transitions (R->N, N->D)
    // ------------------------------------------------------------------

    // @CsvSource -> judged with assertEquals on the resulting gear
    @ParameterizedTest
    @CsvSource({
            "R, N, 0.0",
            "R, N, 50.0",
            "N, D, 0.0",
            "N, D, 80.0"
    })
    void testChangeToSpeedIndependentTransitions(Gear from, Gear to, double speedKmh) {
        moveTo(from);
        stubSpeed(speedKmh);

        gearBox.changeTo(to);

        assertEquals(to, gearBox.getCurrentGear());
    }

    // ------------------------------------------------------------------
    // changeTo: transitions that are never allowed
    // ------------------------------------------------------------------

    // @MethodSource -> judged with assertAll (exception type + message + gear unchanged)
    @ParameterizedTest
    @MethodSource("forbiddenTransitions")
    void testChangeToForbiddenTransitions(Gear from, Gear to) {
        moveTo(from);
        stubSpeed(0.0); // even when stationary these must be rejected

        IllegalGearChangeException thrown = assertThrows(IllegalGearChangeException.class,
                () -> gearBox.changeTo(to));

        assertAll(
                () -> assertTrue(thrown.getMessage().contains(from.toString())),
                () -> assertTrue(thrown.getMessage().contains(to.toString())),
                () -> assertEquals(from, gearBox.getCurrentGear()));
    }

    static Stream<Arguments> forbiddenTransitions() {
        return Stream.of(
                arguments(Gear.P, Gear.N),
                arguments(Gear.P, Gear.D),
                arguments(Gear.R, Gear.P),
                arguments(Gear.N, Gear.P),
                arguments(Gear.N, Gear.R),
                arguments(Gear.D, Gear.P),
                arguments(Gear.D, Gear.N));
    }
}