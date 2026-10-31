package autopilot.core;

public final class SteeringController {
    private static final double PHYSICAL_LIMIT_DEGREES = 450.0;
    private static final double HIGH_SPEED_LIMIT_DEGREES = 90.0;
    private static final double HIGH_SPEED_THRESHOLD_KMH = 120.0;
    private static final double MAX_CHANGE_PER_TICK_DEGREES = 60.0;

    private double currentAngleDegrees;
    private double targetAngleDegrees;

    public double update(double requestedAngleDegrees, double speedKmh) {
        validateAngle(requestedAngleDegrees);
        validateSpeed(speedKmh);
        double allowedLimit = speedKmh > HIGH_SPEED_THRESHOLD_KMH
                ? HIGH_SPEED_LIMIT_DEGREES
                : PHYSICAL_LIMIT_DEGREES;
        targetAngleDegrees = clamp(requestedAngleDegrees, -allowedLimit, allowedLimit);
        return moveTowardTarget();
    }

    public double selfCenter() {
        targetAngleDegrees = 0.0;
        return moveTowardTarget();
    }

    public double getCurrentAngleDegrees() {
        return currentAngleDegrees;
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    private double moveTowardTarget() {
        double difference = targetAngleDegrees - currentAngleDegrees;
        if (Math.abs(difference) <= MAX_CHANGE_PER_TICK_DEGREES) {
            currentAngleDegrees = targetAngleDegrees;
        } else {
            currentAngleDegrees += Math.copySign(MAX_CHANGE_PER_TICK_DEGREES, difference);
        }
        return currentAngleDegrees;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void validateAngle(double angleDegrees) {
        if (!Double.isFinite(angleDegrees)) {
            throw new IllegalArgumentException("requested angle must be finite");
        }
    }

    private void validateSpeed(double speedKmh) {
        if (!Double.isFinite(speedKmh) || speedKmh < 0.0) {
            throw new IllegalArgumentException("speed must be a finite non-negative value");
        }
    }
}
