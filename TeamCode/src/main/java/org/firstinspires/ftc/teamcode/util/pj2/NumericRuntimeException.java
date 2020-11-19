package org.firstinspires.ftc.teamcode.util.pj2;

public class NumericRuntimeException extends RuntimeException{

// Exported constructors.

    /**
     * Construct a new numeric runtime exception with no detail message and no
     * cause.
     */
    public NumericRuntimeException() {
        super();
    }

    /**
     * Construct a new numeric runtime exception with the given detail message
     * and no cause.
     *
     * @param message Detail message.
     */
    public NumericRuntimeException
    (String message) {
        super(message);
    }

    /**
     * Construct a new numeric runtime exception with the given cause and the
     * default detail message.
     *
     * @param cause Cause.
     */
    public NumericRuntimeException
    (Throwable cause) {
        super(cause);
    }

    /**
     * Construct a new numeric runtime exception with the given detail message
     * and the given cause.
     */
    public NumericRuntimeException
    (String message,
     Throwable cause) {
        super(message, cause);
    }


}
