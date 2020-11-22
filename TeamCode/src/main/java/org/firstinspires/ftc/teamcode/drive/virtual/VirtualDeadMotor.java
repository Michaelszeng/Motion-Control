package org.firstinspires.ftc.teamcode.drive.virtual;

import android.os.SystemClock;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.GEAR_RATIO;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.HARDCODED_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_HORIZONTAL_TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_BASE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.WHEEL_RADIUS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.odoEncoderTicksPerRev;

public class VirtualDeadMotor implements DcMotorEx {
    private String TAG = "VirtualDeadMotor";
    private String motor_name;
    private MotorConfigurationType motorType;
    private DcMotor.ZeroPowerBehavior zeroBehave;
    private DcMotor.RunMode runMode;
    private DcMotorSimple.Direction direction;
    private HardwareDevice.Manufacturer manufacturer;
    private DriveTrain drive;
    private MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);
    public static boolean initialized = false;
    public static double WHEEL_RADIUS = 1.25; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed
    public static int ODOMETRY_WHEEL_NUM = 3;
    public static List<Double> newEncoderValues = new ArrayList<Double>(ODOMETRY_WHEEL_NUM);
    public static List<Double> lastEncoderValues = new ArrayList<Double>(ODOMETRY_WHEEL_NUM);
    private static Pose2d lastPose;

    public VirtualDeadMotor(DriveTrain drv, String name)
    {
        drive = drv;
        motor_name = name;
        RobotLogger.dd(TAG, "VirtualDeadMotor constructor " + name + " initialized? " + VirtualDeadMotor.initialized);

        if (VirtualDeadMotor.initialized == false) {
            VirtualDeadMotor.lastPose = new Pose2d(0, 0, 0);
            for (int i = 0; i < ODOMETRY_WHEEL_NUM; i++) {
                VirtualDeadMotor.newEncoderValues.add(i, 0.0);
                VirtualDeadMotor.lastEncoderValues.add(i, 0.0);
            }
            RobotLogger.dd(TAG, "constructed VirtualDeadMotor " + name);
            VirtualDeadMotor.initialized = true;
        }
    }
    public static void resetDeadMotors() {
        VirtualDeadMotor.initialized = false;
    }
    public void setPower(double power) {
        RobotLogger.dd(TAG, "not for dead wheels");
    }
    public double getPower() {
        RobotLogger.dd(TAG, "not for dead wheels");
        return 0;
    }
    private void calculateEncoderValues() {
        // deltaX, deltaY are robot local X, Y delta;
        // delta_x, delta_y are global x, y delta;
        // delta_x, delta_y --> deltaX, deltaY --> rs, rt --> deltaB, deltaR, deltaL;
        long current_time = SystemClock.elapsedRealtime();
        Pose2d current_pos = drive.getRobotPose();
        double delta_x = current_pos.getX() - VirtualDeadMotor.lastPose.getX();
        double delta_y = current_pos.getY() - VirtualDeadMotor.lastPose.getY();
        double delta_h = current_pos.getHeading() - VirtualDeadMotor.lastPose.getHeading();
        double theta = VirtualDeadMotor.lastPose.getHeading();
        double EPSILON = 1e-3, deltaX = 0, deltaY = 0, rs = 0, rt = 0;
        double deltaB = 0, deltaR = 0, deltaL = 0;
        double r = ODOMETRY_TRACK_WIDTH / 2;
        double rb = ODOMETRY_HORIZONTAL_TRACK_WIDTH;

        deltaY = (delta_y * Math.cos(theta - Math.PI/2)) - (delta_x * Math.sin(theta - Math.PI/2));
        deltaY = deltaY / (Math.pow(Math.cos(theta - Math.PI/2), 2)  + Math.pow(Math.sin(theta - Math.PI/2), 2));

        deltaX = delta_y  - deltaY * Math.cos(theta - Math.PI/2);
        deltaX = deltaX / Math.sin(theta - Math.PI/2);

        double diff_h = Math.abs(delta_h);

        diff_h = delta_h % (2 * Math.PI);
        diff_h = (diff_h + 2 * Math.PI) % (2 * Math.PI);
        if (diff_h > Math.PI)
            diff_h = Math.abs(diff_h - 2 * Math.PI);
        RobotLogger.dd(TAG, "calculateEncoderValues deltaX: " + deltaX + " deltaY: " + deltaY + " delta_h: " + delta_h  + " diff_h: " + diff_h);

        delta_h = diff_h;
        if (Math.abs(diff_h) <= EPSILON) {
            deltaB = deltaX;
            deltaR = deltaY + r * delta_h;
            deltaL = deltaY - r * delta_h;
            RobotLogger.dd(TAG, "calculateEncoderValues (no turn) deltaL: " + deltaR + " deltaL: " + deltaL + " deltaB: " + deltaB);
        }
        else {
            rs = deltaX * Math.sin(delta_h) - deltaY * (Math.cos(delta_h) -1);
            rs = rs / (Math.pow(Math.cos(delta_h) - 1, 2) + Math.pow(Math.sin(delta_h), 2));
            rt = (deltaX - rs * Math.sin(delta_h))/(Math.cos(delta_h) - 1);

            deltaB = rs * delta_h + rb * delta_h;
            deltaL = rt * delta_h - r * delta_h;
            deltaR = 2 * r * delta_h + deltaL;
            RobotLogger.dd(TAG, "calculateEncoderValues (turn) deltaL: " + deltaR + " deltaL: " + deltaL + " deltaB: " + deltaB);
        }

        double new_left = deltaL + VirtualDeadMotor.lastEncoderValues.get(0);
        double new_right = deltaR + VirtualDeadMotor.lastEncoderValues.get(1);
        double new_front = deltaB + VirtualDeadMotor.lastEncoderValues.get(2);

        // in inches
        VirtualDeadMotor.newEncoderValues.clear();
        VirtualDeadMotor.newEncoderValues.add(0, new_left);
        VirtualDeadMotor.newEncoderValues.add(1, new_right);
        VirtualDeadMotor.newEncoderValues.add(2, new_front);

        RobotLogger.dd(TAG, "calculateEncoderValues current time: " + current_time + " last pose: " + lastPose);
        RobotLogger.dd(TAG, "calculateEncoderValues current time: " + current_time + " current pose: " + current_pos);
        RobotLogger.dd(TAG, "calculateEncoderValues previous odom(inches): " + VirtualDeadMotor.lastEncoderValues.get(0)
                + " " + VirtualDeadMotor.lastEncoderValues.get(1) + " " + VirtualDeadMotor.lastEncoderValues.get(2));
        RobotLogger.dd(TAG, "calculateEncoderValues current odom(inches): " + new_left + " " + new_right + " " + new_front);

        lastPose = current_pos;
        VirtualDeadMotor.lastEncoderValues.clear();
        VirtualDeadMotor.lastEncoderValues.add(0, new_left);
        VirtualDeadMotor.lastEncoderValues.add(1, new_right);
        VirtualDeadMotor.lastEncoderValues.add(2, new_front);
    }
    private int getEncoderValueFromIndex(int index) {
        //RobotLogger.dd(TAG, "list size: " + VirtualDeadMotor.newEncoderValues.size());
        Double t = VirtualDeadMotor.newEncoderValues.get(index);
        t = inchesToTicks(t);
        return t.intValue();
    }
    /**
     * Sets the velocity of the motor
     * @param angularRate  the desired ticks per second
     */
    public void setVelocity(double angularRate) {
        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    /**
     * Sets the velocity of the motor
     * @param angularRate   the desired angular rate, in units per second
     * @param unit          the units in which angularRate is expressed
     *
     * @see #getVelocity(AngleUnit)
     */
    public void setVelocity(double angularRate, AngleUnit unit) {
        RobotLogger.callers(2, TAG, "not implemented yet");
    };

    private double inchesToTicks(double t) {
        double r = odoEncoderTicksPerRev * t / (WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO);
        return r;
    }
    /**
     * Returns the current velocity of the motor, in ticks per second
     * @return the current velocity of the motor
     */
    public double getVelocity() {
        double r = 0;
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
        RobotLogger.callers(2, TAG, "not implemented yet");
        return 0;
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
    };

    /**
     * Returns whether the motor is currently in a float power level.
     * @return whether the motor is currently in a float power level.
     * @see #setPowerFloat()
     */
    public boolean getPowerFloat() {
        return false;};

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

    private int getMotorIndexFromName(String name) {
        if (name == "leftEncoder")
            return 0;
        if (name == "rightEncoder")
            return 1;
        if (name == "frontEncoder")
            return 2;
        else
            return 0;
    }
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
        int index = getMotorIndexFromName(motor_name);
        if (index == 0) {
            calculateEncoderValues();
        }
        int current_pos = getEncoderValueFromIndex(index);
        RobotLogger.dd(TAG, "Odometry wheel name: " + motor_name + " current time: " + current_time + " current encoder value: " + current_pos);
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

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode mode) {
        PIDCoefficients t = new PIDCoefficients();
        return t;
    };
    public PIDFCoefficients getPIDFCoefficients(DcMotor.RunMode mode) {
        PIDFCoefficients t = new PIDFCoefficients();
        return t;
    };
    public void setPositionPIDFCoefficients(double p) {};
    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {};
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {};
    public void setPIDCoefficients(DcMotor.RunMode mode, PIDCoefficients pidCoefficients) {};
    public boolean isMotorEnabled() {return true;};
    public void setMotorDisable() {};
    public void setMotorEnable() {};
}
