package autopilot.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;

import autopilot.core.AdaptiveCruise.Mode;
import autopilot.core.AutoDriver.DriveResult;
import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;

class AutoDriverTest {

    private static final double TARGET_SPEED_KMH = 80.0;
    private static final double DELTA = 1e-9;

    // What each stub sensor reports. A test changes only the source it cares about;
    // a source may also throw to simulate a failing sensor.
    private DoubleSupplier speedSource;
    private DoubleSupplier distanceSource;
    private DoubleSupplier laneSource;

    // Hand-written test doubles that delegate to the sources above.
    private final SpeedSensor speedSensor = new SpeedSensor() {
        @Override
        public double getSpeedKmh() {
            return speedSource.getAsDouble();
        }
    };
    private final ObstacleSensor obstacleSensor = new ObstacleSensor() {
        @Override
        public double getDistanceMeters() {
            return distanceSource.getAsDouble();
        }
    };
    private final LaneSensor laneSensor = new LaneSensor() {
        @Override
        public double getLaneOffsetMeters() {
            return laneSource.getAsDouble();
        }
    };

    // Used by the @EnumSource test: which sensor is failing.
    private enum FailingSensor {
        SPEED, OBSTACLE, LANE
    }

    // Fresh, healthy "normal driving" fixture for every test:
    // at target speed, clear road, centred in lane. Each test overrides only what it needs.
    @BeforeEach
    void setUp() {
        speedSource = () -> TARGET_SPEED_KMH;
        distanceSource = () -> 100.0;
        laneSource = () -> 0.0;
    }

    // Each test builds its own driver, so no state is shared between tests.
    private AutoDriver newDriver() {
        return new AutoDriver(speedSensor, obstacleSensor, laneSensor, TARGET_SPEED_KMH);
    }

    // ------------------------------------------------------------------
    // Normal driving (replaces test1)
    // Expected: speed == target and clear road -> HOLD, no brake, no steering,
    // no warning, not in safe mode.
    // ------------------------------------------------------------------

    @Test
    @Tag("cruise")
    @DisplayName("At target speed on a clear, centred road the car holds: no brake, no steering, no warning")
    void testNormalDrivingHoldsWithoutBrakeOrSteering() {
        DriveResult result = newDriver().tick(GearBox.Gear.D);

        assertEquals(new DriveResult(0.0, 0.0, Mode.HOLD, false, false), result);
    }

    // ------------------------------------------------------------------
    // Lane keeping (replaces testLanePositive / testLaneNegative)
    // Threshold is 0.5 m: strictly greater deviation triggers a 45 degree correction
    // in the opposite direction; anything inside [-0.5, 0.5] keeps the wheel centred.
    // Values sit on both sides of each boundary (boundary value analysis).
    // ------------------------------------------------------------------

    @ParameterizedTest
    @CsvSource({
            "1.0,   -45.0",   // clearly right of centre -> steer negative
            "0.51,  -45.0",   // just above +threshold   -> steer negative
            "0.5,   0.0",     // exactly +threshold      -> no correction
            "0.0,   0.0",     // centred                 -> no correction
            "-0.5,  0.0",     // exactly -threshold      -> no correction
            "-0.51, 45.0",    // just below -threshold   -> steer positive
            "-1.0,  45.0"     // clearly left of centre  -> steer positive
    })
    @Tag("lane-keeping")
    void testLaneCorrectionSteeringByOffset(double laneOffsetMeters, double expectedSteeringDegrees) {
        laneSource = () -> laneOffsetMeters;

        DriveResult result = newDriver().tick(GearBox.Gear.D);

        assertEquals(expectedSteeringDegrees, result.steeringAngleDegrees(), DELTA);
    }

    // ------------------------------------------------------------------
    // Sensor failure (replaces testSpeedSensorFailure / testObstacleSensorFailure)
    // Any failing sensor -> safe mode: no steering, 30 % brake, BRAKE mode, no warning.
    // High-risk behaviour, so it is tagged "safety" for risk-based test selection.
    // ------------------------------------------------------------------

    @ParameterizedTest
    @EnumSource(FailingSensor.class)
    @Tag("safety")
    void testSafeModeWhenASensorFails(FailingSensor failingSensor) {
        SensorFailureException failure = new SensorFailureException("sensor failed");
        switch (failingSensor) {
            case SPEED -> speedSource = () -> {
                throw failure;
            };
            case OBSTACLE -> distanceSource = () -> {
                throw failure;
            };
            case LANE -> laneSource = () -> {
                throw failure;
            };
        }

        DriveResult result = newDriver().tick(GearBox.Gear.D);

        assertEquals(new DriveResult(0.0, 30.0, Mode.BRAKE, false, true), result);
    }
}