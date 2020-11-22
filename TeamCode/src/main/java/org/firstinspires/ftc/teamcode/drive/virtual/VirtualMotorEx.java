package org.firstinspires.ftc.teamcode.drive.virtual;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HARDCODED_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hP;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kP;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.txD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.txI;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.txP;

public class VirtualMotorEx implements DcMotorEx {
    private String TAG = "VirtualMotorEx";
    private MotorConfigurationType motorType;
    private DcMotor.ZeroPowerBehavior zeroBehave;
    private DcMotor.RunMode runMode;
    private DcMotorSimple.Direction direction;
    private HardwareDevice.Manufacturer manufacturer;
    private boolean float_power = false;
    private boolean motor_enabled = false;
    private String motor_name;
    private double motor_power = 0;
    private PIDFCoefficients PIDF_parameters;
    private PIDCoefficients PID_parameters;
    private DriveTrain driveTrain;
    private long last_get_wheel_position_time = 0;
    private double last_get_wheel_position_power = 0;
    private int last_wheel_position = 0;
    /* hard code to be GoBILDA5202 type, need to be careful not to use the parameters from xml */
    private MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);

    public VirtualMotorEx(MecanumDrive drv, String name)
    {
        motor_name = name;
        last_get_wheel_position_time = SystemClock.elapsedRealtime();
//        RobotLogger.dd(TAG, "create VirtualMotorEx " + name + " " + last_get_wheel_position_time);
        last_get_wheel_position_power = 0;
        last_wheel_position = 0;
        motor_power = 0;
    }
    public void setPower(double power) {
//        RobotLogger.dd(TAG, "%s: setPower %f", motor_name, power);
        motor_power = power;
    }

    /**
     * Individually energizes this particular motor
     * @see #setMotorDisable()
     * @see #isMotorEnabled()
     */
    public void setMotorEnable() {
        motor_enabled = true;
    };

    /**
     * Individually de-energizes this particular motor
     * @see #setMotorEnable()
     * @see #isMotorEnabled()
     */
    public void setMotorDisable() {
        motor_enabled = false;
    };

    /**
     * Returns whether this motor is energized
     * @see #setMotorEnable()
     * @see #setMotorDisable()
     */
    public boolean isMotorEnabled() {
//        RobotLogger.callers(2, TAG, "not implemented yet");
        return motor_enabled;
    };

    /**
     * Sets the velocity of the motor
     * @param angularRate  the desired ticks per second
     */
    public void setVelocity(double angularRate) {
//        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     * @see #getVelocity(AngleUnit)
     */
    public void setVelocity(double angularRate, AngleUnit unit) {
//        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    private int inchesToTicks(double t) {
        int r = (int)(HARDCODED_TICKS_PER_REV * t / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO));
        return r;
    }
    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        double r = last_get_wheel_position_power / kV;
        r = inchesToTicks(r);
        last_get_wheel_position_power = motor_power;
        //RobotLogger.dd(TAG, motor_name + ", getVelocity: " + Double.toString(r) + " last_get_wheel_position_power: " + Double.toString(last_get_wheel_position_power));
        return r;
    }

    /**
     * Returns the current velocity of the motor, in angular units per second
     * @param unit          the units in which the angular rate is desired
     * @return              the current velocity of the motor
     *
     * @see #setVelocity(double, AngleUnit)
     */
    public double getVelocity(AngleUnit unit) {
//        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
    };

    /**
     * Sets the PID control coefficients for one of the PID modes of this motor.
     * Note that in some controller implementations, setting the PID coefficients for one
     * mode on a motor might affect other modes on that motor, or might affect the PID
     * coefficients used by other motors on the same controller (this is not true on the
     * REV Expansion Hub).
     *
     * @param mode either {@link RunMode#RUN_USING_ENCODER} or {@link RunMode#RUN_TO_POSITION}
     * @param pidCoefficients the new coefficients to use when in that mode on this motor
     *
     * @see #getPIDCoefficients(RunMode)
     *
     * @deprecated Use {@link #setPIDFCoefficients(RunMode, PIDFCoefficients)} instead
     */
    @Deprecated
    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {
        PID_parameters = pidCoefficients;
//        RobotLogger.dd(TAG, "setPIDFCoefficients %f, %f, %f",
//                PID_parameters.p, PID_parameters.i, PID_parameters.d);
    };

    /**
     * {@link #setPIDFCoefficients} is a superset enhancement to {@link #setPIDCoefficients}. In addition
     * to the proportional, integral, and derivative coefficients previously supported, a feed-forward
     * coefficient may also be specified. Further, a selection of motor control algorithms is offered:
     * the originally-shipped Legacy PID algorithm, and a PIDF algorithm which avails itself of the
     * feed-forward coefficient. Note that the feed-forward coefficient is not used by the Legacy PID
     * algorithm; thus, the feed-forward coefficient must be indicated as zero if the Legacy PID
     * algorithm is used. Also: the internal implementation of these algorithms may be different: it
     * is not the case that the use of PIDF with the F term as zero necessarily exhibits exactly the
     * same behavior as the use of the LegacyPID algorithm, though in practice they will be quite close.
     *
     * Readers are reminded that {@link DcMotor.RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPositionPIDFCoefficients(double)
     * @see #getPIDFCoefficients(RunMode)
     */
    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        PIDF_parameters = pidfCoefficients;
//        RobotLogger.dd(TAG, "PIDF: %f, %f, %f, %f",
//                PIDF_parameters.p, PIDF_parameters.i, PIDF_parameters.d, PIDF_parameters.f);
    };

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotor.RunMode#RUN_USING_ENCODER}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
//        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    /**
     * A shorthand for setting the PIDF coefficients for the {@link DcMotor.RunMode#RUN_TO_POSITION}
     * mode. {@link MotorControlAlgorithm#PIDF} is used.
     *
     * Readers are reminded that {@link DcMotor.RunMode#RUN_TO_POSITION} mode makes use of <em>both</em>
     * the coefficients set for RUN_TO_POSITION <em>and</em> the coefficients set for RUN_WITH_ENCODER,
     * due to the fact that internally the RUN_TO_POSITION logic calculates an on-the-fly velocity goal
     * on each control cycle, then (logically) runs the RUN_WITH_ENCODER logic. Because of that double-
     * layering, only the proportional ('p') coefficient makes logical sense for use in the RUN_TO_POSITION
     * coefficients.
     *
     * @see #setVelocityPIDFCoefficients(double, double, double, double)
     * @see #setPIDFCoefficients(RunMode, PIDFCoefficients)
     */
    public void setPositionPIDFCoefficients(double p) {

        RobotLogger.callers(2, TAG, "not implemented yet");

    };


    /**
     * Returns the PID control coefficients used when running in the indicated mode
     * on this motor.
     * @param mode either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @return the PID control coefficients used when running in the indicated mode on this motor
     *
     * @deprecated Use {@link #getPIDFCoefficients(DcMotor.RunMode)} instead
     */
    @Deprecated
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode mode) {
        return PID_parameters;
    };

    /**
     * Returns the PIDF control coefficients used when running in the indicated mode
     * on this motor.
     * @param mode either {@link DcMotor.RunMode#RUN_USING_ENCODER} or {@link DcMotor.RunMode#RUN_TO_POSITION}
     * @return the PIDF control coefficients used when running in the indicated mode on this motor
     *
     * @see #setPIDFCoefficients(DcMotor.RunMode, PIDFCoefficients)
     */
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        return (PIDF_parameters);
    };
    /**
     * Sets the target positioning tolerance of this motor
     * @param tolerance the desired tolerance, in encoder ticks
     * @see DcMotor#setTargetPosition(int)
     */
    public void setTargetPositionTolerance(int tolerance) {
        RobotLogger.callers(2, TAG, "not implemented yet");

    };

    /**
     * Returns the current target positioning tolerance of this motor
     * @return the current target positioning tolerance of this motor
     */
    public int getTargetPositionTolerance() {

        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
    };

    /**
     * Returns the current consumed by this motor.
     * @param unit current units
     * @return the current consumed by this motor.
     */
    public double getCurrent(CurrentUnit unit) {
        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
    };

    /**
     * Returns the current alert for this motor.
     * @param unit current units
     * @return the current alert for this motor
     */
    public double getCurrentAlert(CurrentUnit unit) {

        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
    };

    /**
     * Sets the current alert for this motor
     * @param current current alert
     * @param unit current units
     */
    public void setCurrentAlert(double current, CurrentUnit unit) {
        RobotLogger.callers(2, TAG, "not implemented yet");

    };

    /**
     * Returns whether the current consumption of this motor exceeds the alert threshold.
     * @return whether the current consumption of this motor exceeds the alert threshold.
     */
    public boolean isOverCurrent() {

        RobotLogger.callers(2, TAG, "not implemented yet");
        return false;
    }

    /**
     * Returns the assigned type for this motor. If no particular motor type has been
     * configured, then {@link MotorConfigurationType#getUnspecifiedMotorType()} will be returned.
     * Note that the motor type for a given motor is initially assigned in the robot
     * configuration user interface, though it may subsequently be modified using methods herein.
     * @return the assigned type for this motor
     */
    public MotorConfigurationType getMotorType() {
        return MOTOR_CONFIG;
    };

    /**
     * Sets the assigned type of this motor. Usage of this method is very rare.
     * @param motorType the new assigned type for this motor
     * @see #getMotorType()
     */
    public void setMotorType(MotorConfigurationType motorType) {this.motorType = motorType;}

    /**
     * Returns the underlying motor controller on which this motor is situated.
     * @return the underlying motor controller on which this motor is situated.
     * @see #getPortNumber()
     */
    public DcMotorController getController() {
        RobotLogger.callers(2, TAG, "not implemented yet");

        return null;
    };

    /**
     * Returns the port number on the underlying motor controller on which this motor is situated.
     * @return the port number on the underlying motor controller on which this motor is situated.
     * @see #getController()
     */
    public int getPortNumber() {

        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
    };

    /**
     * ZeroPowerBehavior provides an indication as to a motor's behavior when a power level of zero
     * is applied.
     * @see #setZeroPowerBehavior(DcMotor.ZeroPowerBehavior)
     * @see #setPower(double)
     */
    enum ZeroPowerBehavior
    {
        /** The behavior of the motor when zero power is applied is not currently known. This value
         * is mostly useful for your internal state variables. It may not be passed as a parameter
         * to {@link #setZeroPowerBehavior(DcMotor.ZeroPowerBehavior)} and will never be returned from
         * {@link #getZeroPowerBehavior()}*/
        UNKNOWN,
        /** The motor stops and then brakes, actively resisting any external force which attempts
         * to turn the motor. */
        BRAKE,
        /** The motor stops and then floats: an external force attempting to turn the motor is not
         * met with active resistence. */
        FLOAT
    }

    /**
     * Sets the behavior of the motor when a power level of zero is applied.
     * @param zeroPowerBehavior the new behavior of the motor when a power level of zero is applied.
     * @see DcMotor.ZeroPowerBehavior
     * @see #setPower(double)
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        zeroBehave = zeroPowerBehavior;
    };

    /**
     * Returns the current behavior of the motor were a power level of zero to be applied.
     * @return the current behavior of the motor were a power level of zero to be applied.
     */
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroBehave;
    };

    /**
     * Sets the zero power behavior of the motor to {@link DcMotor.ZeroPowerBehavior#FLOAT FLOAT}, then
     * applies zero power to that motor.
     *
     * <p>Note that the change of the zero power behavior to {@link DcMotor.ZeroPowerBehavior#FLOAT FLOAT}
     * remains in effect even following the return of this method. <STRONG>This is a breaking
     * change</STRONG> in behavior from previous releases of the SDK. Consider, for example, the
     * following code sequence:</p>
     *
     * <pre>
     *     motor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE) {}; // method not available in previous releases
     *     motor.setPowerFloat() {};
     *     motor.setPower(0.0) {};
     * </pre>
     *
     * <p>Starting from this release, this sequence of code will leave the motor floating. Previously,
     * the motor would have been left braked.</p>
     *
     * @see #setPower(double)
     * @see #getPowerFloat()
     * @see #setZeroPowerBehavior(DcMotor.ZeroPowerBehavior)
     * @deprecated This method is deprecated in favor of direct use of
     *       {@link #setZeroPowerBehavior(DcMotor.ZeroPowerBehavior) setZeroPowerBehavior()} and
     *       {@link #setPower(double) setPower()}.
     */
    public @Deprecated void setPowerFloat() {
        float_power = true;
    };

    /**
     * Returns whether the motor is currently in a float power level.
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    public boolean getPowerFloat() {
        return float_power;};

    /**
     * Sets the desired encoder target position to which the motor should advance or retreat
     * and then actively hold thereat. This behavior is similar to the operation of a servo.
     * The maximum speed at which this advance or retreat occurs is governed by the power level
     * currently set on the motor. While the motor is advancing or retreating to the desired
     * taget position, {@link #isBusy()} will return true.
     *
     * <p>Note that adjustment to a target position is only effective when the motor is in
     * {@link DcMotor.RunMode#RUN_TO_POSITION RUN_TO_POSITION}
     * RunMode. Note further that, clearly, the motor must be equipped with an encoder in order
     * for this mode to function properly.</p>
     *
     * @param position the desired encoder target position
     * @see #getCurrentPosition()
     * @see #setMode(DcMotor.RunMode)
     * @see DcMotor.RunMode#RUN_TO_POSITION
     * @see #getTargetPosition()
     * @see #isBusy()
     */
    public void setTargetPosition(int position) {
        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    /**
     * Returns the current target encoder position for this motor.
     * @return the current target encoder position for this motor.
     * @see #setTargetPosition(int)
     */
    public int getTargetPosition() {
        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;};

    /**
     * Returns true if the motor is currently advancing or retreating to a target position.
     * @return true if the motor is currently advancing or retreating to a target position.
     * @see #setTargetPosition(int)
     */
    public boolean isBusy() {
        RobotLogger.callers(2, TAG, "not implemented yet");
        return false;};

    /**
     * Returns the current reading of the encoder for this motor. The units for this reading,
     * that is, the number of ticks per revolution, are specific to the motor/encoder in question,
     * and thus are not specified here.
     * @return the current reading of the encoder for this motor
     * @see #getTargetPosition()
     * @see DcMotor.RunMode#STOP_AND_RESET_ENCODER
     */
    public int getCurrentPosition() {
        long current_time = SystemClock.elapsedRealtime();
        long time_duration = current_time - last_get_wheel_position_time;
        double v = getVelocity();
        double delta = v * ((double)time_duration) / 1000.0;
//        RobotLogger.dd(TAG, "current_time: " + current_time + " last_get_wheel_position_time: " + last_get_wheel_position_time + " velocity: " + Double.toString(v) + " delta position: " + Double.toString(delta));

        int current_pos =  (int) (delta + last_wheel_position);

//        RobotLogger.dd(TAG, motor_name + ", velocity: " + Double.toString(v)
//                + ", time duration: " + Double.toString(time_duration) + ", currPosition: "
//                + Double.toString(current_pos) + ", last power: " + Double.toString(last_get_wheel_position_power)
//                + ", current power: " + Double.toString(motor_power));
        last_get_wheel_position_time = current_time;
        last_wheel_position = current_pos;
        return current_pos;
    };

    /**
     * The run mode of a motor {@link DcMotor.RunMode} controls how the motor interprets the
     * it's parameter settings passed through power- and encoder-related methods.
     * Some of these modes internally use <a href="https://en.wikipedia.org/wiki/PID_controller">PID</a>
     * control to achieve their function, while others do not. Those that do are referred
     * to as "PID modes".
     */
    enum RunMode
    {
        /** The motor is simply to run at whatever velocity is achieved by apply a particular
         * power level to the motor.
         */
        RUN_WITHOUT_ENCODER,

        /** The motor is to do its best to run at targeted velocity. An encoder must be affixed
         * to the motor in order to use this mode. This is a PID mode.
         */
        RUN_USING_ENCODER,

        /** The motor is to attempt to rotate in whatever direction is necessary to cause the
         * encoder reading to advance or retreat from its current setting to the setting which
         * has been provided through the {@link #setTargetPosition(int) setTargetPosition()} method.
         * An encoder must be affixed to this motor in order to use this mode. This is a PID mode.
         */
        RUN_TO_POSITION,

        /** The motor is to set the current encoder position to zero. In contrast to
         * {@link com.qualcomm.robotcore.hardware.DcMotor.RunMode#RUN_TO_POSITION RUN_TO_POSITION},
         * the motor is not rotated in order to achieve this; rather, the current rotational
         * position of the motor is simply reinterpreted as the new zero value. However, as
         * a side effect of placing a motor in this mode, power is removed from the motor, causing
         * it to stop, though it is unspecified whether the motor enters brake or float mode.
         *
         * Further, it should be noted that setting a motor to{@link DcMotor.RunMode#STOP_AND_RESET_ENCODER
         * STOP_AND_RESET_ENCODER} may or may not be a transient state: motors connected to some motor
         * controllers will remain in this mode until explicitly transitioned to a different one, while
         * motors connected to other motor controllers will automatically transition to a different
         * mode after the reset of the encoder is complete.
         */
        STOP_AND_RESET_ENCODER,

        /** @deprecated Use {@link #RUN_WITHOUT_ENCODER} instead */
        @Deprecated RUN_WITHOUT_ENCODERS,

        /** @deprecated Use {@link #RUN_USING_ENCODER} instead */
        @Deprecated RUN_USING_ENCODERS,

        /** @deprecated Use {@link #STOP_AND_RESET_ENCODER} instead */
        @Deprecated RESET_ENCODERS;



        /**
         * Returns whether this RunMode is a PID-controlled mode or not
         * @return whether this RunMode is a PID-controlled mode or not
         */
        public boolean isPIDMode()
        {
            return this==RUN_USING_ENCODER || this==RUN_USING_ENCODERS || this==RUN_TO_POSITION;
        }
    }

    /**
     * Sets the current run mode for this motor
     * @param mode the new current run mode for this motor
     * @see DcMotor.RunMode
     * @see #getMode()
     */
    public void setMode(DcMotor.RunMode mode) { runMode = mode;};

    /**
     * Returns the current run mode for this motor
     * @return the current run mode for this motor
     * @see DcMotor.RunMode
     * @see #setMode(DcMotor.RunMode)
     */
    public DcMotor.RunMode getMode() { return runMode;
    };

    /* DcMotorSimple */
    /**
     * DcMotors can be configured to internally reverse the values
     * to which, e.g., their motor power is set. This makes it easy
     * to have drive train motors on two sides of a robot: during initialization,
     * one would be set at at forward, the other at reverse, and the
     * difference between the two in that respect could be thereafter ignored.
     *
     * <p>At the start of an OpMode, motors are guaranteed to be in the forward direction.</p>
     *
     * @see #setDirection(DcMotorSimple.Direction)
     */
    enum Direction { FORWARD, REVERSE;
        public Direction inverted() {
            return this==FORWARD ? REVERSE : FORWARD;
        }
    }

    /**
     * Sets the logical direction in which this motor operates.
     * @param direction the direction to set for this motor
     *
     * @see #getDirection()
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    /**
     * Returns the current logical direction in which this motor is set as operating.
     * @return the current logical direction in which this motor is set as operating.
     * @see #setDirection(DcMotorSimple.Direction)
     */
    public DcMotorSimple.Direction getDirection() {
        return direction;
        };


    /**
     * Returns the current configured power level of the motor.
     * @return the current level of the motor, a value in the interval [0.0, 1.0]
     * @see #setPower(double)
     */
    public double getPower() {
//        RobotLogger.dd(TAG, "getPower %f", motor_power);
        return motor_power;
        };
    /*  */

    enum Manufacturer {
        Unknown, Other, Lego, HiTechnic, ModernRobotics, Adafruit, Matrix, Lynx, AMS, STMicroelectronics, Broadcom
    }

    /**
     * Returns an indication of the manufacturer of this device.
     * @return the device's manufacturer
     */
    public HardwareDevice.Manufacturer getManufacturer() {
        return manufacturer;

    };

    /**
     * Returns a string suitable for display to the user as to the type of device.
     * Note that this is a device-type-specific name; it has nothing to do with the
     * name by which a user might have configured the device in a robot configuration.
     *
     * @return device manufacturer and name
     */
    public String getDeviceName() {
            return ("VirtualMotoroEx");
    };

    /**
     * Get connection information about this device in a human readable format
     *
     * @return connection info
     */
    public String getConnectionInfo() {
        return ("good");
    };

    /**
     * Version
     *
     * @return get the version of this device
     */
    public int getVersion() {
        return (1);
    };

    /**
     * Resets the device's configuration to that which is expected at the beginning of an OpMode.
     * For example, motors will reset the their direction to 'forward'.
     */
    public void resetDeviceConfigurationForOpMode() {
        RobotLogger.callers(2, TAG, "not implemented yet");

    };

    /**
     * Closes this device
     */
    public void close() {

    };

}

