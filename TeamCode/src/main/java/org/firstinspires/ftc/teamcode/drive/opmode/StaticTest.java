package org.firstinspires.ftc.teamcode.drive.opmode;

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
This class slowly increases the power to the wheels until the robot is able to move (Detected by
odometry wheels). This find the minimum power required for movement.
 */
@TeleOp(name = "StaticTest")
//@Disabled
public class StaticTest extends LinearOpMode {
    String TAG = "StaticTest";
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    BNO055IMU imu;
    double power;
    double minPower;

    Date datePrev = new Date();
    Date dateNew = new Date();
    double dateDiff;

    double currentSpeed;
    double currentSpeedIn;
    double prevBackRight=0;
    double PrevBackLeft=0;
    final double revsPerIn = 1/7.42107;

    @Override
    public void runOpMode() throws InterruptedException {
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

        power = 0.02;
        minPower = 0.0;

        waitForStart();

        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());

            robot.setMotorPowers(power, power, power, power);

            currentSpeed = ((Math.abs(backRight.getCurrentPosition()-prevBackRight)/(dateDiff/1000)) + (Math.abs(backLeft.getCurrentPosition()-PrevBackLeft)/(dateDiff/1000)))/2;
            currentSpeedIn = currentSpeed / (odoTicksPerRevRight * revsPerIn);
            RobotLogger.dd(TAG, "current speed ticks: " + currentSpeed);
            RobotLogger.dd(TAG, "current speed inches: " + currentSpeedIn);

//            if ((Math.abs(backRight.getCurrentPosition()) < 100) && Math.abs(backLeft.getCurrentPosition()) < 100) {
            if (currentSpeedIn < 0.1) {
                power += 0.004;
            }
            else {
                minPower = power;
            }

            telemetry.addData("Current Power: ", power);
            telemetry.addData("Min Power: ", minPower);
            telemetry.addData("Right Odometer", backRight.getCurrentPosition());
            telemetry.addData("Left Odometer", backLeft.getCurrentPosition());
            telemetry.addData("Horizontal Odometer", frontRight.getCurrentPosition());
            telemetry.update();

            try {
                Thread.sleep(1250);
            }
            catch (Exception e) {

            }

            prevBackRight = backRight.getCurrentPosition();
            PrevBackLeft = backLeft.getCurrentPosition();

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
