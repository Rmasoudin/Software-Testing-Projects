package autopilot.exception;

public class SensorFailureException extends RuntimeException {
    public SensorFailureException(String message) {
        super(message);
    }
}
