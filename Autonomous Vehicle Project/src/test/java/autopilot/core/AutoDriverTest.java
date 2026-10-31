package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class AutoDriverTest {

    private SpeedSensor speedSensor;
    private ObstacleSensor obstacleSensor;
    private LaneSensor laneSensor;
    private AutoDriver autoDriver;

    @BeforeEach
    void setUp() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                return 80.0;
            }
        };

        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                return 100.0;
            }
        };

        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return 0.0;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor,
                obstacleSensor,
                laneSensor,
                80.0
        );
    }

    @Test
    void test1() {
        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertNotNull(result);
        assertFalse(result.safeMode());
        assertNotNull(result.cruiseMode());
        assertEquals(0.0, result.brakePressure());
    }

    @Test
    void testLanePositive() {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return 1.0;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, 80.0
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() < 0.0);
    }

    @Test
    void testLaneNegative() {
        laneSensor = new LaneSensor() {
            @Override
            public double getLaneOffsetMeters() {
                return -1.0;
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, 80.0
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.steeringAngleDegrees() > 0.0);
    }

    @Test
    void testSpeedSensorFailure() {
        speedSensor = new SpeedSensor() {
            @Override
            public double getSpeedKmh() {
                throw new SensorFailureException("Speed sensor failed");
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, 80.0
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.safeMode());
        assertEquals(30.0, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }

    @Test
    void testObstacleSensorFailure() {
        obstacleSensor = new ObstacleSensor() {
            @Override
            public double getDistanceMeters() {
                throw new SensorFailureException("Obstacle sensor failed");
            }
        };

        autoDriver = new AutoDriver(
                speedSensor, obstacleSensor, laneSensor, 80.0
        );

        AutoDriver.DriveResult result = autoDriver.tick(GearBox.Gear.D);

        assertTrue(result.safeMode());
        assertEquals(30.0, result.brakePressure());
        assertEquals(0.0, result.steeringAngleDegrees());
        assertEquals(AdaptiveCruise.Mode.BRAKE, result.cruiseMode());
        assertFalse(result.warningActive());
    }
}
