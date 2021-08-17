package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.io.File;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

@TeleOp(name = "CurrentDrawTest")
public class CurrentDrawTest extends LinearOpMode {
    private String TAG = "CurrentDrawTest";
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    BNO055IMU imu;

    Date datePrev = new Date();
    Date dateNew = new Date();
    double dateDiff;

    double power;
    double prevY;

    FtcDashboard dashboard;

    String powerString = "";
    String currentStringSingle = "";
    String currentStringAvg = "";
    File logFile = AppUtil.getInstance().getSettingsFile("currentVoltageData.txt");

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new AllHardwareMap(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        frontRight = hwMap.frontRight;
        backRight = hwMap.backRight;
        frontLeft = hwMap.frontLeft;
        backLeft = hwMap.backLeft;

        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initHardwareMap(frontRight, backRight, frontLeft, backLeft);

        imu = hwMap.gyro;

        //Initialize IMU parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        Robot robot = new Robot(hardwareMap, false, 0.0, 0.0, 0.0);
        ArrayList<Double> currentDraws;

        power = 0.02;
        prevY = 0.0;

        waitForStart();

        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());

            double imuReading = -imu.getAngularOrientation().firstAngle;
            robot.update(backRight.getCurrentPosition(), backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), imuReading);
            Pose2d currentPose = robot.getCurrentPose();
            double currentV = (currentPose.getY() - prevY)/dateDiff;

            double scaledPose = currentPose.getY()/20;
            double scaledV = currentV * 50;

            currentDraws = robot.getCurrentDraws();


            telemetry.addData("Power: ", power);
            telemetry.addData("FR Current: ", currentDraws.get(2));
            telemetry.addData("BR Current: ", currentDraws.get(3));
            telemetry.addData("BL Current: ", currentDraws.get(1));
            telemetry.addData("FL Current: ", currentDraws.get(0));

            powerString += power;
            powerString += ",";
            currentStringSingle += currentDraws.get(2);
            currentStringSingle += ",";
            currentStringAvg += calculateAverage(currentDraws);
            currentStringAvg += ",";

//            telemetry.addData("Localizer: ", "(" + currentPose.getX() + ", " + currentPose.getY() + ", " + currentPose.getHeading() + ")");
//            telemetry.addData("CurrentY / 20: ", scaledPose);
//            telemetry.addData("CurrentV * 50: ", scaledV);
            telemetry.update();

            robot.setMotorPowers(power, power, power, power);
            power += 0.01;
            prevY = currentPose.getY();

            if (power > 1.25) {
                exportTelemetryData();
                this.stop();
            }

            try {
                Thread.sleep(25);
            }
            catch (Exception e) {

            }

            idle();
        }
    }


    private void initHardwareMap(DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back){
//        right_front = hardwareMap.dcMotor.get(rfName);
//        right_back = hardwareMap.dcMotor.get(rbName);
//        left_front = hardwareMap.dcMotor.get(lfName);
//        left_back = hardwareMap.dcMotor.get(lbName);

//        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
//        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private double calculateAverage(List<Double> marks) {
        double sum = 0;
        if(!marks.isEmpty()) {
            for (double mark : marks) {
                sum += mark;
            }
            return sum / marks.size();
        }
        return sum;
    }

    private void exportTelemetryData() {
        String masterString = powerString + "\n" + "*" + "\n" + currentStringAvg + "\n" + "&" + "\n" + currentStringSingle;
        ReadWriteFile.writeFile(logFile, masterString);
        RobotLogger.dd(TAG, "Successfully wrote to telemetry file.");
    }
}
