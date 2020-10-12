package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.control.PIDController;
import org.firstinspires.ftc.teamcode.drive.virtual.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.virtual.VirtualLocalizer;
import org.firstinspires.ftc.teamcode.drive.virtual.VirtualMotorEx;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.BASE_CONSTRAINTS;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.getMotorVelocityF;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

//import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

public class Robot extends MecanumDrive {
    boolean RUN_USING_ENCODER = false;
    ThreeOdometerLocalizer localizer;
    ArrayList<PIDController> pidControllers = new ArrayList<>();
    private DriveConstraints constraints;
    private String TAG = "Robot";

    private FtcDashboard dashboard;
    private NanoClock clock;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }
    private Mode mode;
    private List<Pose2d> poseHistory = new ArrayList<>();
    private List<Pose2d> targetHistory = new ArrayList<>();
    private double prevLeftOdoReading = 0.0;
    private double prevRightOdoReading = 0.0;
    private double prevHorizontalOdoReading = 0.0;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private boolean virtual;
    // added for drive simulator
    private DriveTrain _virtualDriveTrain;

    public Robot(HardwareMap hardwareMap, boolean virtual, double startX, double startY, double startHeading) {
        super(kV, kA, kStatic, TRACK_WIDTH);
        constraints = new MecanumConstraints(BASE_CONSTRAINTS, TRACK_WIDTH);
        localizer = new ThreeOdometerLocalizer();
        poseHistory.add(new Pose2d(startX, startY, startHeading));
        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        this.virtual = virtual;

//        this.setPoseEstimate(new Pose2d(startX, startY, startHeading));

        if (!DriveConstants.VirtualizeDrive) {
            LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }

            // TODO: adjust the names of the following hardware devices to match your configuration
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
            // upward (normal to the floor) using a command like the following:
            // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

            leftFront = hardwareMap.get(DcMotorEx.class, "frontLeft");
            leftRear = hardwareMap.get(DcMotorEx.class, "backLeft");
            rightRear = hardwareMap.get(DcMotorEx.class, "backRight");
            rightFront = hardwareMap.get(DcMotorEx.class, "frontRight");
            motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        } else {
            leftFront = new VirtualMotorEx(this, "leftFront");
            leftRear = new VirtualMotorEx(this, "leftRear");
            rightRear = new VirtualMotorEx(this, "rightRear");
            rightFront = new VirtualMotorEx(this, "rightFront");
            motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
            _virtualDriveTrain = new DriveTrain(this);
            _virtualDriveTrain.AddMotors(motors);
            setLocalizer(new VirtualLocalizer(_virtualDriveTrain));
            //setLocalizer(new MecanumLocalizer(this, false));
//            RobotLogger.dd(TAG, "use default 4 wheel localizer");
        }

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }
        // TODO: reverse any motors using DcMotor.setDirection()
    }

    //Physical Robot: Just Localization
    public void update(double rightOdoReading, double leftOdoReading, double horizontalOdoReading, double headingReading) {
        double leftOdoChange = leftOdoReading - prevLeftOdoReading;
        double rightOdoChange = rightOdoReading - prevRightOdoReading;
        double horizontalOdoChange = horizontalOdoReading - prevHorizontalOdoReading;
//        RobotLogger.dd(TAG, "leftOdoChange: " + leftOdoChange);
//        RobotLogger.dd(TAG, "rightOdoChange: " + rightOdoChange);
//        RobotLogger.dd(TAG, "horizOdoChange: " + horizontalOdoChange);
        double previousHeading = poseHistory.get(poseHistory.size() - 1).getHeading();
        ArrayList<Double> coordinateChange = localizer.getPoseChangeV1(leftOdoChange, rightOdoChange, horizontalOdoChange, headingReading, previousHeading);
//        ArrayList<Double> coordinateChange = localizer.getPoseChangeV2(leftOdoChange, rightOdoChange, horizontalOdoChange, headingReading, previousHeading);
//        ArrayList<Double> coordinateChange = localizer.getPoseChangeV3(leftOdoChange, rightOdoChange, horizontalOdoChange, leftOdoReading, rightOdoReading, horizontalOdoReading, headingReading, previousHeading);

        Pose2d prevPose = poseHistory.get(poseHistory.size() - 1);
        Pose2d currentPose = new Pose2d(prevPose.getX() + coordinateChange.get(0), prevPose.getY() + coordinateChange.get(1), headingReading);
        poseHistory.add(currentPose);

        prevLeftOdoReading = leftOdoReading;
        prevRightOdoReading = rightOdoReading;
        prevHorizontalOdoReading = horizontalOdoReading;
    }

    //Physical Robot: Follow Point
    public ArrayList<Double> update(double rightOdoReading, double leftOdoReading, double horizontalOdoReading, double headingReading, Pose2d targetPose, int loopTime) {
        double leftOdoChange = leftOdoReading - prevLeftOdoReading;
        double rightOdoChange = rightOdoReading - prevRightOdoReading;
        double horizontalOdoChange = horizontalOdoReading - prevHorizontalOdoReading;
        double previousHeading = poseHistory.get(poseHistory.size() - 1).getHeading();
//        double headingChange = headingReading - previousHeading;

        ArrayList<Double> coordinateChange = localizer.getPoseChangeV1(leftOdoChange, rightOdoChange, horizontalOdoChange, headingReading, previousHeading);
//        ArrayList<Double> coordinateChange = localizer.getPoseChangeV2(leftOdoChange, rightOdoChange, horizontalOdoChange, headingReading, previousHeading);
//        ArrayList<Double> coordinateChange = localizer.getPoseChangeV3(leftOdoChange, rightOdoChange, horizontalOdoChange, leftOdoReading, rightOdoReading, horizontalOdoReading, headingReading, previousHeading);

        Pose2d prevPose = poseHistory.get(poseHistory.size() - 1);
        Pose2d currentPose = new Pose2d(prevPose.getX() + coordinateChange.get(0), prevPose.getY() + coordinateChange.get(1), headingReading);
        poseHistory.add(currentPose);
        targetHistory.add(targetPose);

        ArrayList<Double> motorPowers;  //Fl, BL, BR, FR
        Pose2d prevTargetPose;

        if (targetHistory.size() > 1) {
            prevTargetPose = targetHistory.get(targetHistory.size() - 2);
        }
        else {
            prevTargetPose = targetPose;
        }

        RobotLogger.dd(TAG, "prevTargetPose: (" + prevTargetPose.getX() + ", " + prevTargetPose.getY() + ", " + prevTargetPose.getHeading() + ")");
        RobotLogger.dd(TAG, "targetPose: (" + targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getHeading() + ")");

        if (pidControllers.size() > 0 && targetPose.getX() == prevTargetPose.getX() && targetPose.getY() == prevTargetPose.getY() && targetPose.getHeading() == prevTargetPose.getHeading()) {
            PIDController latestController = pidControllers.get(pidControllers.size() - 1);
            motorPowers = latestController.update(currentPose, loopTime);
        }
        else {
            PIDController controller = new PIDController(currentPose, targetPose, 24.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 1.0, 1.2);
            pidControllers.add(controller);
            motorPowers = controller.update(currentPose, loopTime);
        }


//        ArrayList<Double> motorPowers;  //Fl, BL, BR, FR
//        if (targetHistory.get(targetHistory.size() - 1).equals(targetHistory.get(targetHistory.size() - 2))) {
//            motorPowers = pidControllers.get(pidControllers.size() - 1).update(currentPose, loopTime);
//        }
//        else {
//            PIDController controller = new PIDController(currentPose, targetPose, 24.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
//            pidControllers.add(controller);
//            motorPowers = controller.update(currentPose, loopTime);
//        }

        prevLeftOdoReading = leftOdoReading;
        prevRightOdoReading = rightOdoReading;
        prevHorizontalOdoReading = horizontalOdoReading;

        return motorPowers;
    }

    //Virtual Robot: Follow Point
    public ArrayList<Double> update(Pose2d currentPose, Pose2d targetPose, int loopTime) {
        //The old pose's heading sometimes has an extra 2Pi added
        poseHistory.add(currentPose);
        targetHistory.add(targetPose);

        ArrayList<Double> motorPowers;  //Fl, BL, BR, FR
        Pose2d prevTargetPose;

        if (targetHistory.size() > 1) {
            prevTargetPose = targetHistory.get(targetHistory.size() - 2);
        }
        else {
            prevTargetPose = targetPose;
        }

        RobotLogger.dd(TAG, "prevTargetPose: (" + prevTargetPose.getX() + ", " + prevTargetPose.getY() + ", " + prevTargetPose.getHeading() + ")");
        RobotLogger.dd(TAG, "targetPose: (" + targetPose.getX() + ", " + targetPose.getY() + ", " + targetPose.getHeading() + ")");

        if (pidControllers.size() > 0 && targetPose.getX() == prevTargetPose.getX() && targetPose.getY() == prevTargetPose.getY() && targetPose.getHeading() == prevTargetPose.getHeading()) {
            PIDController latestController = pidControllers.get(pidControllers.size() - 1);
            motorPowers = latestController.update(currentPose, loopTime);
        }
        else {
            PIDController controller = new PIDController(currentPose, targetPose, 24.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.5, 1.0, 1.2);
            pidControllers.add(controller);
            motorPowers = controller.update(currentPose, loopTime);
        }
        RobotLogger.dd(TAG, "pidControllers.size(): " + pidControllers.size());
        return motorPowers;
    }

    public Pose2d getCurrentPose() {
        return poseHistory.get(poseHistory.size() - 1);
    }

//    public void waitForIdle() {
//        while (!Thread.currentThread().isInterrupted() && isBusy()) {
//            update();
//        }
//    }

    public boolean isBusy() {
        return mode != Robot.Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        RobotLogger.dd(TAG, "setPIDCoefficients");
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, getMotorVelocityF()
            ));
        }
    }

    @Override
    public List<Double> getWheelPositions() {
        RobotLogger.dd(TAG, "getWheelPositions");
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        RobotLogger.dd(TAG, "getWheelVelocities");

        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
//        RobotLogger.dd(TAG,"setMotorPowers");

        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        RobotLogger.callers(4, TAG, "getRawExternalHeading");
        if (!DriveConstants.VirtualizeDrive) {
            return imu.getAngularOrientation().firstAngle;
        } else {
            Pose2d pose = _virtualDriveTrain.getRobotPose();
//            RobotLogger.dd(TAG, "Simulated Pose (IMU ExternalHeading): " + pose.toString());
            return pose.getHeading();
        }
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }

    public List<Double> getOdomWheelPositions() {
        RobotLogger.dd(TAG, "getOdomWheelPositions");
        List<Double> wheelPositions = new ArrayList<>();
        List<DcMotorEx> motors = _virtualDriveTrain.getOdomMotors();
        for (DcMotorEx motor : motors) {
            int pos = motor.getCurrentPosition();
            double t = StandardTrackingWheelLocalizer.encoderTicksToInches(pos);
            RobotLogger.dd(TAG, "getOdomWheelPositions, motor position(ticks): " + pos + "  ticks to inches: " + t);
            wheelPositions.add(t);
        }
        return wheelPositions;
    }
}
