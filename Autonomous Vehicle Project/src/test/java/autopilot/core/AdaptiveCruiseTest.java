package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
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

import autopilot.core.AdaptiveCruise.Decision;
import autopilot.core.AdaptiveCruise.Mode;

class AdaptiveCruiseTest {

    private static final double TARGET_SPEED = 100.0;
    private static final double DELTA = 1e-9;

    private AdaptiveCruise cruise;

    // Fresh fixture for every test: no shared state between tests.
    @BeforeEach
    void setUp() {
        cruise = new AdaptiveCruise(TARGET_SPEED);
    }

    // ------------------------------------------------------------------
    // safeDistanceFor
    // ------------------------------------------------------------------

    // @CsvSource -> judged with assertEquals + delta (numeric results)
    @ParameterizedTest
    @CsvSource({
            "0.0,   0.0",
            "0.5,   0.25",
            "40.0,  20.0",
            "100.0, 50.0"
    })
    void testSafeDistanceForValidSpeeds(double speedKmh, double expectedMeters) {
        assertEquals(expectedMeters, cruise.safeDistanceFor(speedKmh), DELTA);
    }

    @Test
    void testSafeDistanceRejectsNegativeSpeed() {
        assertThrows(IllegalArgumentException.class, () -> cruise.safeDistanceFor(-1.0));
    }

    // ------------------------------------------------------------------
    // update: input validation
    // ------------------------------------------------------------------

    // @ValueSource -> judged with assertThrows (invalid inputs)
    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testUpdateRejectsInvalidSpeed(double invalidSpeed) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(invalidSpeed, 50.0));
    }

    @ParameterizedTest
    @ValueSource(doubles = { -1.0, -0.001, Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY })
    void testUpdateRejectsInvalidDistance(double invalidDistance) {
        assertThrows(IllegalArgumentException.class, () -> cruise.update(50.0, invalidDistance));
    }

    // ------------------------------------------------------------------
    // update: emergency threshold (boundary value analysis)
    // ------------------------------------------------------------------

    // @CsvFileSource -> judged with assertTrue / assertFalse (classification)
    @ParameterizedTest
    @CsvFileSource(resources = "/emergency_threshold.csv", numLinesToSkip = 1)
    void testEmergencyClassificationAroundThreshold(double speedKmh, double distanceMeters,
            boolean expectedEmergency) {
        boolean isEmergency = cruise.update(speedKmh, distanceMeters).mode() == Mode.EMERGENCY_BRAKE;

        if (expectedEmergency) {
            assertTrue(isEmergency);
        } else {
            assertFalse(isEmergency);
        }
    }

    // ------------------------------------------------------------------
    // update: decisions when the car was NOT braking before
    // ------------------------------------------------------------------

    // @MethodSource -> judged with assertAll on mode AND amount (complex result)
    @ParameterizedTest
    @MethodSource("decisionsWithoutPriorBraking")
    void testUpdateDecisionWithoutPriorBraking(double speedKmh, double distanceMeters,
            Mode expectedMode, double expectedAmount) {
        Decision decision = cruise.update(speedKmh, distanceMeters);

        assertAll(
                () -> assertEquals(expectedMode, decision.mode()),
                () -> assertEquals(expectedAmount, decision.amount(), DELTA));
    }

    static Stream<Arguments> decisionsWithoutPriorBraking() {
        // target speed = 100 km/h in every case
        return Stream.of(
                // proportional brake: safe = 50 m, emergency limit = 25 m
                arguments(100.0, 40.0, Mode.BRAKE, 20.0),
                arguments(100.0, 49.5, Mode.BRAKE, 1.0),
                // accelerate: capped at the 5 km/h step
                arguments(60.0, 100.0, Mode.ACCELERATE, 5.0),
                // accelerate: limited by remaining gap to target speed
                arguments(98.0, 100.0, Mode.ACCELERATE, 2.0),
                // accelerate from standstill (safe distance 0, distance > 0)
                arguments(0.0, 5.0, Mode.ACCELERATE, 5.0),
                // release band (safe 40 m, release 48 m) is NOT sticky without prior braking
                arguments(80.0, 44.0, Mode.ACCELERATE, 5.0),
                // hold: distance exactly equals safe distance
                arguments(60.0, 30.0, Mode.HOLD, 0.0),
                // hold: already at target speed
                arguments(100.0, 200.0, Mode.HOLD, 0.0),
                // hold: above target speed
                arguments(120.0, 200.0, Mode.HOLD, 0.0));
    }

    // ------------------------------------------------------------------
    // update: every mode honours its amount contract
    // ------------------------------------------------------------------

    // @EnumSource -> judged with assertSame (mode identity) + assertTrue (amount range)
    @ParameterizedTest
    @EnumSource(Mode.class)
    void testDecisionAmountRangeForEachMode(Mode mode) {
        Decision decision = switch (mode) {
            case ACCELERATE -> cruise.update(60.0, 100.0);
            case HOLD -> cruise.update(100.0, 200.0);
            case BRAKE -> cruise.update(100.0, 40.0);
            case EMERGENCY_BRAKE -> cruise.update(100.0, 10.0);
        };

        boolean amountInRange = switch (mode) {
            case ACCELERATE -> decision.amount() > 0.0 && decision.amount() <= 5.0;
            case HOLD -> decision.amount() == 0.0;
            case BRAKE -> decision.amount() > 0.0 && decision.amount() < 100.0;
            case EMERGENCY_BRAKE -> decision.amount() == 100.0;
        };

        assertSame(mode, decision.mode());
        assertTrue(amountInRange);
    }

    // ------------------------------------------------------------------
    // update: hysteresis (stateful behaviour, speed 80 -> safe 40 m,
    // emergency limit 20 m, release distance 48 m)
    // ------------------------------------------------------------------

    @Test
    void testBrakeContinuesInReleaseBandAfterBraking() {
        cruise.update(80.0, 35.0); // enter braking

        Decision decision = cruise.update(80.0, 44.0); // safe <= d < release

        assertAll(
                () -> assertEquals(Mode.BRAKE, decision.mode()),
                () -> assertEquals(10.0, decision.amount(), DELTA));
    }

    @Test
    void testBrakePressureTracksDistanceWhileStillBraking() {
        cruise.update(80.0, 35.0); // enter braking (12.5)

        Decision decision = cruise.update(80.0, 30.0); // still below safe distance

        assertAll(
                () -> assertEquals(Mode.BRAKE, decision.mode()),
                () -> assertEquals(25.0, decision.amount(), DELTA));
    }

    @Test
    void testBrakeReleasesAtReleaseDistance() {
        cruise.update(80.0, 35.0); // enter braking

        Decision atRelease = cruise.update(80.0, 48.0); // exactly the release distance
        Decision afterRelease = cruise.update(80.0, 44.0); // band is no longer sticky

        assertAll(
                () -> assertEquals(Mode.ACCELERATE, atRelease.mode()),
                () -> assertEquals(Mode.ACCELERATE, afterRelease.mode()));
    }

    @Test
    void testEmergencyBrakeLatchesBrakingState() {
        cruise.update(80.0, 10.0); // emergency (<= 20 m)

        Decision decision = cruise.update(80.0, 44.0); // release band

        assertAll(
                () -> assertEquals(Mode.BRAKE, decision.mode()),
                () -> assertEquals(10.0, decision.amount(), DELTA));
    }
}
