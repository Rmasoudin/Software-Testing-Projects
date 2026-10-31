package autopilot.core;

import autopilot.exception.IllegalGearChangeException;
import autopilot.sensor.SpeedSensor;

import java.util.Objects;

public final class GearBox {
    public enum Gear { P, R, N, D }

    private static final double REVERSE_ENTRY_SPEED_LIMIT_KMH = 5.0;
    private Gear currentGear = Gear.P;
    private SpeedSensor speedSensor;

    public GearBox(SpeedSensor speedSensor) {
        this.speedSensor = Objects.requireNonNull(speedSensor, "speedSensor must not be null");
    }

    public Gear getCurrentGear() {
        return currentGear;
    }

    public void setSpeedSensor(SpeedSensor speedSensor) {
        this.speedSensor = Objects.requireNonNull(speedSensor, "speedSensor must not be null");
    }

    public void changeTo(Gear targetGear) {
        Objects.requireNonNull(targetGear, "targetGear must not be null");
        if (targetGear == currentGear) {
            return;
        }

        double speedKmh = speedSensor.getSpeedKmh();
        validateSpeed(speedKmh);
        if (!isAllowed(currentGear, targetGear, speedKmh)) {
            throw new IllegalGearChangeException(
                    "Cannot change from " + currentGear + " to " + targetGear + " at " + speedKmh + " km/h");
        }
        currentGear = targetGear;
    }

    private boolean isAllowed(Gear from, Gear to, double speedKmh) {
        if (from == Gear.P && to == Gear.R) {
            return speedKmh < REVERSE_ENTRY_SPEED_LIMIT_KMH;
        }
        if (from == Gear.R && to == Gear.D) {
            return speedKmh == 0.0;
        }
        if (from == Gear.D && to == Gear.R) {
            return speedKmh == 0.0;
        }
        return (from == Gear.R && to == Gear.N)
                || (from == Gear.N && to == Gear.D);
    }

    private void validateSpeed(double speedKmh) {
        if (!Double.isFinite(speedKmh) || speedKmh < 0.0) {
            throw new IllegalArgumentException("speed must be a finite non-negative value");
        }
    }
}
