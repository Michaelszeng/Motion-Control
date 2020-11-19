package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.ArrayList;
import java.util.Date;

/*
This class makes the robot accelerate rapidly from rest.
Given the velocity at every moment, the program subtracts measuredVelocity/kV to get the power for acceleration only.
The acceleration is then measured, and plotted against power in the dashboard, and the slope is the kA.
 */
@TeleOp(name = "kATest")
//@Disabled
public class kATest extends LinearOpMode {
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    BNO055IMU imu;
    double power;
    double prevY;
    double prevV;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        hwMap = new AllHardwareMap(hardwareMap);

        frontRight = hwMap.frontRight;
        backRight = hwMap.backRight;
        frontLeft = hwMap.frontLeft;
        backLeft = hwMap.backLeft;

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(frontRight, backRight, frontLeft, backLeft);

        imu = hwMap.gyro;

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        Robot robot = new Robot(hardwareMap, false, 0.0, 0.0, 0.0);

        NanoClock clock = NanoClock.system();

        Date datePrev = new Date();
        Date dateNew = new Date();
        double dateDiff;

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        power = 0.02;
        prevY = 0.0;
        prevV = 0.0;

        waitForStart();

        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());

            double imuReading = -imu.getAngularOrientation().firstAngle;
            robot.update(backRight.getCurrentPosition(), backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), imuReading);
            Pose2d currentPose = robot.getCurrentPose();
            double currentV = (currentPose.getY() - prevY)/dateDiff;
            double currentA = (currentV - prevV)/dateDiff;

            double scaledPose = currentPose.getY()/20;
            double scaledV = currentV * 50;
            double scaledA = currentA * Math.pow(10, 4);

            double aOnlyP = power - (currentV / DriveConstants.kVM);

            telemetry.addData("Current Power: ", power);
            telemetry.addData("CurrentY / 20: ", scaledPose);
            telemetry.addData("CurrentV * 50: ", scaledV);
            telemetry.addData("CurrentA * 10^4: ", scaledA);
            telemetry.addData("Acceleration Only Power: ", aOnlyP);
            telemetry.update();

            robot.setMotorPowers(power, power, power, power);
            power *= 1.1;
            prevY = currentPose.getY();
            prevV = currentV;

            try {
                Thread.sleep(25);
            }
            catch (Exception e) {

            }

            idle();
        }
    }

    private void initHardwareMap(DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back){
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
