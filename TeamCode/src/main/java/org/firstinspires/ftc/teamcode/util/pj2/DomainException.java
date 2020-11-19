package org.firstinspires.ftc.teamcode.util.pj2;

public class DomainException
        extends NumericRuntimeException
{

// Exported constructors.

    /**
     * Construct a new domain exception with no detail message and no cause.
     */
    public DomainException()
    {
        super();
    }

    /**
     * Construct a new domain exception with the given detail message and no
     * cause.
     *
     * @param  message  Detail message.
     */
    public DomainException
    (String message)
    {
        super (message);
    }

    /**
     * Construct a new domain exception with the given cause and the default
     * detail message.
     *
     * @param  cause  Cause.
     */
    public DomainException
    (Throwable cause)
    {
        super (cause);
    }

    /**
     * Construct a new domain exception with the given detail message and the
     * given cause.
     */
    public DomainException
    (String message,
     Throwable cause)
    {
        super (message, cause);
    }
}
