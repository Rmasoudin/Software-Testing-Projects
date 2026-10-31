package autopilot.core;

import autopilot.exception.SensorFailureException;
import autopilot.sensor.LaneSensor;
import autopilot.sensor.ObstacleSensor;
import autopilot.sensor.SpeedSensor;

import java.util.Objects;

public final class AutoDriver {
    private static final double LANE_DEVIATION_THRESHOLD_METERS = 0.5;
    private static final double LANE_CORRECTION_DEGREES = 45.0;
    private static final double SAFE_MODE_BRAKE_PRESSURE = 30.0;

    public record DriveResult(double steeringAngleDegrees,
                              double brakePressure,
                              AdaptiveCruise.Mode cruiseMode,
                              boolean warningActive,
                              boolean safeMode) {
    }

    private final SpeedSensor speedSensor;
    private final ObstacleSensor obstacleSensor;
    private final LaneSensor laneSensor;
    private final SteeringController steeringController;
    private final BrakeController brakeController;
    private final AdaptiveCruise adaptiveCruise;

    public AutoDriver(SpeedSensor speedSensor,
                      ObstacleSensor obstacleSensor,
                      LaneSensor laneSensor,
                      double targetSpeedKmh) {
        this.speedSensor = Objects.requireNonNull(speedSensor, "speedSensor must not be null");
        this.obstacleSensor = Objects.requireNonNull(obstacleSensor, "obstacleSensor must not be null");
        this.laneSensor = Objects.requireNonNull(laneSensor, "laneSensor must not be null");
        this.steeringController = new SteeringController();
        this.brakeController = new BrakeController();
        this.adaptiveCruise = new AdaptiveCruise(targetSpeedKmh);
    }

    public DriveResult tick(GearBox.Gear gear) {
        try {
            double speedKmh = speedSensor.getSpeedKmh();
            double distanceMeters = obstacleSensor.getDistanceMeters();
            double laneOffsetMeters = laneSensor.getLaneOffsetMeters();

            double steeringAngle = updateSteering(laneOffsetMeters, speedKmh);
            AdaptiveCruise.Decision decision = adaptiveCruise.update(speedKmh, distanceMeters);
            double brakePressure = applyCruiseDecision(decision, speedKmh, gear);

            return new DriveResult(
                    steeringAngle,
                    brakePressure,
                    decision.mode(),
                    brakeController.isWarningActive(),
                    false);
        } catch (SensorFailureException failure) {
            return new DriveResult(
                    0.0,
                    SAFE_MODE_BRAKE_PRESSURE,
                    AdaptiveCruise.Mode.BRAKE,
                    false,
                    true);
        }
    }

    private double updateSteering(double laneOffsetMeters, double speedKmh) {
        if (laneOffsetMeters > LANE_DEVIATION_THRESHOLD_METERS) {
            return steeringController.update(-LANE_CORRECTION_DEGREES, speedKmh);
        }
        if (laneOffsetMeters < -LANE_DEVIATION_THRESHOLD_METERS) {
            return steeringController.update(LANE_CORRECTION_DEGREES, speedKmh);
        }
        return steeringController.selfCenter();
    }

    private double applyCruiseDecision(AdaptiveCruise.Decision decision,
                                       double speedKmh,
                                       GearBox.Gear gear) {
        return switch (decision.mode()) {
            case BRAKE -> brakeController.apply(decision.amount(), speedKmh, gear);
            case EMERGENCY_BRAKE -> {
                brakeController.emergencyBrake(gear);
                yield brakeController.getPressure();
            }
            case ACCELERATE, HOLD -> 0.0;
        };
    }
}
