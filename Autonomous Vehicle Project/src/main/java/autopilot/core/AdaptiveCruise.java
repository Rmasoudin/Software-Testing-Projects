package autopilot.core;

public final class AdaptiveCruise {
    private static final double METERS_PER_KMH = 0.5;
    private static final double EMERGENCY_DISTANCE_RATIO = 0.5;
    private static final double RELEASE_DISTANCE_RATIO = 1.2;
    private static final double MAX_ACCELERATION_STEP_KMH = 5.0;
    private static final double HYSTERESIS_BRAKE_PRESSURE = 10.0;

    public enum Mode {
        ACCELERATE,
        HOLD,
        BRAKE,
        EMERGENCY_BRAKE
    }

    public record Decision(Mode mode, double amount) {
    }

    private final double targetSpeedKmh;
    private boolean braking;

    public AdaptiveCruise(double targetSpeedKmh) {
        if (!Double.isFinite(targetSpeedKmh) || targetSpeedKmh < 0.0) {
            throw new IllegalArgumentException("target speed must be a finite non-negative value");
        }
        this.targetSpeedKmh = targetSpeedKmh;
    }

    public double safeDistanceFor(double speedKmh) {
        validateSpeed(speedKmh);
        return speedKmh * METERS_PER_KMH;
    }

    public Decision update(double speedKmh, double distanceMeters) {
        validateSpeed(speedKmh);
        validateDistance(distanceMeters);

        double safeDistance = safeDistanceFor(speedKmh);
        double emergencyDistance = safeDistance * EMERGENCY_DISTANCE_RATIO;
        double releaseDistance = safeDistance * RELEASE_DISTANCE_RATIO;

        if (distanceMeters <= emergencyDistance) {
            braking = true;
            return new Decision(Mode.EMERGENCY_BRAKE, 100.0);
        }

        if (braking) {
            if (distanceMeters < releaseDistance) {
                return new Decision(Mode.BRAKE,
                        distanceMeters < safeDistance
                                ? proportionalBrake(safeDistance, distanceMeters)
                                : HYSTERESIS_BRAKE_PRESSURE);
            }
            braking = false;
        }

        if (distanceMeters < safeDistance) {
            braking = true;
            return new Decision(Mode.BRAKE, proportionalBrake(safeDistance, distanceMeters));
        }
        if (distanceMeters > safeDistance && speedKmh < targetSpeedKmh) {
            return new Decision(Mode.ACCELERATE,
                    Math.min(MAX_ACCELERATION_STEP_KMH, targetSpeedKmh - speedKmh));
        }
        return new Decision(Mode.HOLD, 0.0);
    }

    private double proportionalBrake(double safeDistance, double distanceMeters) {
        if (safeDistance == 0.0) {
            return 100.0;
        }
        return (safeDistance - distanceMeters) / safeDistance * 100.0;
    }

    private void validateSpeed(double speedKmh) {
        if (!Double.isFinite(speedKmh) || speedKmh < 0.0) {
            throw new IllegalArgumentException("speed must be a finite non-negative value");
        }
    }

    private void validateDistance(double distanceMeters) {
        if (!Double.isFinite(distanceMeters) || distanceMeters < 0.0) {
            throw new IllegalArgumentException("distance must be a finite non-negative value");
        }
    }
}
