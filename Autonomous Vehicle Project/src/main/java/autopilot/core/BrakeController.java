package autopilot.core;

import java.util.Objects;

public final class BrakeController {
    private static final double MAX_PRESSURE = 100.0;
    private static final double ABS_PRESSURE_THRESHOLD = 80.0;
    private static final double ABS_SPEED_THRESHOLD_KMH = 30.0;
    private static final double ABS_HIGH_PULSE = 90.0;
    private static final double ABS_LOW_PULSE = 60.0;

    private double pressure;
    private boolean warningActive;
    private boolean nextAbsPulseIsHigh = true;

    public double apply(double requestedPressure, double speedKmh, GearBox.Gear gear) {
        validatePressure(requestedPressure);
        validateSpeed(speedKmh);
        ensureBrakingAllowed(gear);

        double boundedPressure = Math.min(MAX_PRESSURE, requestedPressure);
        if (boundedPressure > ABS_PRESSURE_THRESHOLD && speedKmh > ABS_SPEED_THRESHOLD_KMH) {
            pressure = nextAbsPulseIsHigh ? ABS_HIGH_PULSE : ABS_LOW_PULSE;
            nextAbsPulseIsHigh = !nextAbsPulseIsHigh;
        } else {
            pressure = boundedPressure;
            nextAbsPulseIsHigh = true;
        }
        return pressure;
    }

    public void emergencyBrake(GearBox.Gear gear) {
        ensureBrakingAllowed(gear);
        pressure = MAX_PRESSURE;
        warningActive = true;
        nextAbsPulseIsHigh = true;
    }

    public double getPressure() {
        return pressure;
    }

    public boolean isWarningActive() {
        return warningActive;
    }

    private void ensureBrakingAllowed(GearBox.Gear gear) {
        if (Objects.requireNonNull(gear, "gear must not be null") == GearBox.Gear.P) {
            throw new IllegalStateException("braking is unavailable while the gearbox is in park");
        }
    }

    private void validatePressure(double requestedPressure) {
        if (!Double.isFinite(requestedPressure) || requestedPressure < 0.0) {
            throw new IllegalArgumentException("pressure must be a finite non-negative value");
        }
    }

    private void validateSpeed(double speedKmh) {
        if (!Double.isFinite(speedKmh) || speedKmh < 0.0) {
            throw new IllegalArgumentException("speed must be a finite non-negative value");
        }
    }
}
