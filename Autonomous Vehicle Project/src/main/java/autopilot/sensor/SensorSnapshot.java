package autopilot.sensor;

import autopilot.exception.SensorFailureException;

import java.util.Objects;

public final class SensorSnapshot implements SpeedSensor, ObstacleSensor, LaneSensor {
    private final Double speedKmh;
    private final Double distanceMeters;
    private final Double laneOffsetMeters;
    private final String failureMessage;

    private SensorSnapshot(Double speedKmh, Double distanceMeters, Double laneOffsetMeters,
                           String failureMessage) {
        this.speedKmh = speedKmh;
        this.distanceMeters = distanceMeters;
        this.laneOffsetMeters = laneOffsetMeters;
        this.failureMessage = failureMessage;
    }

    public static SensorSnapshot of(double speedKmh, double distanceMeters, double laneOffsetMeters) {
        return new SensorSnapshot(speedKmh, distanceMeters, laneOffsetMeters, null);
    }

    public static SensorSnapshot failed(String message) {
        return new SensorSnapshot(null, null, null,
                Objects.requireNonNull(message, "message must not be null"));
    }

    @Override
    public double getSpeedKmh() {
        ensureOperational();
        return speedKmh;
    }

    @Override
    public double getDistanceMeters() {
        ensureOperational();
        return distanceMeters;
    }

    @Override
    public double getLaneOffsetMeters() {
        ensureOperational();
        return laneOffsetMeters;
    }

    private void ensureOperational() {
        if (failureMessage != null) {
            throw new SensorFailureException(failureMessage);
        }
    }
}
