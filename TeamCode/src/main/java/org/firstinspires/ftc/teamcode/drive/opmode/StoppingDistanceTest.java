package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.Date;

@TeleOp(name = "StoppingDistanceTest")
public class StoppingDistanceTest extends LinearOpMode {
    String TAG = "StoppingDistanceTest";
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    BNO055IMU imu;

    double duration = 0.0;
    Date datePrev = new Date();
    Date dateNew = new Date();
    Date startDate = new Date();
    double dateDiff;

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
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        Robot robot = new Robot(hardwareMap, false,0.0, 0.0, 0.0);

        waitForStart();
        startDate.setTime(new Date().getTime());
        while(opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());
            duration = dateNew.getTime() - startDate.getTime();

            if (duration < 2000) {
                robot.setMotorPowers(1.0, 1.0, 1.0, 1.0);
            }
            else {
                robot.setMotorPowers(0.0, 0.0, 0.0, 0.0);
            }


            telemetry.update();
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

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
