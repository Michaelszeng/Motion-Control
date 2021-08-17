package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.Date;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.odoTicksPerRevRight;

/*
This class slowly increases the power to the wheels until the robot is able to strafe (Detected by
odometry wheels). This find the minimum power required for horizontal movement.
 */
@TeleOp(name = "StaticStrafeTest")
//@Disabled
public class StaticStrafeTest extends LinearOpMode {
    String TAG = "StaticStrafeTest";
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    FtcDashboard dashboard;

    BNO055IMU imu;
    double power;
    double minPower;

    Date datePrev = new Date();
    Date dateNew = new Date();
    double dateDiff;

    double currentSpeed;
    double currentSpeedIn;
    double prevOdo=0;
    final double revsPerIn = 1/7.42107;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new AllHardwareMap(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(20);
        TelemetryPacket packet = new TelemetryPacket();

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

        power = 0.08;
        minPower = 0.0;

        waitForStart();

        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());

            robot.setMotorPowers(power, -power, power, -power);

            currentSpeed = Math.abs(frontRight.getCurrentPosition()-prevOdo)/(dateDiff/1000);
            currentSpeedIn = currentSpeed / (odoTicksPerRevRight * revsPerIn);
            RobotLogger.dd(TAG, "current speed ticks: " + currentSpeed);
            RobotLogger.dd(TAG, "current speed inches: " + currentSpeedIn);

            if (currentSpeedIn < 0.1) {
                power += 0.005;
            }
            else {
                minPower = power;
            }

            telemetry.addData("Current Power: ", power);
            telemetry.addData("Min Power: ", minPower);
            telemetry.addData("Horizontal Odometer", frontRight.getCurrentPosition());
            telemetry.update();

            packet.put("speed", currentSpeedIn);
            dashboard.sendTelemetryPacket(packet);

            try {
                Thread.sleep(1000);
            }
            catch (Exception e) {

            }

            prevOdo = frontRight.getCurrentPosition();

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
