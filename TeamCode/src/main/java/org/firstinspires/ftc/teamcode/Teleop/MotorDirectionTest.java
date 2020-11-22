package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

@TeleOp(name = "MotorDirectionTest")
public class MotorDirectionTest extends LinearOpMode {
    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    BNO055IMU imu;

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

        while(opModeIsActive()) {   //put teleop code in here
//            telemetry.addData("Right Vertical: ", verticalRight.getCurrentPosition());
//            telemetry.addData("Left Vertical: ", verticalLeft.getCurrentPosition());
//            telemetry.addData("horizontal: ", horizontal.getCurrentPosition());
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);

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
}
