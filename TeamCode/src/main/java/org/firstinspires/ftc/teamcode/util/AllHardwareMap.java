package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.virtual.VirtualMotorEx;

public class AllHardwareMap {
    public DcMotorEx backLeft, backRight, frontLeft, frontRight, leftIntake, rightIntake, liftOne, liftTwo;
    public Servo clawServo1, clawServo2, foundationLock, transferLock, transferHorn,
            clawInit, innerTransfer, parkingServo, redAutoClawJoint1, redAutoClawJoint2, redAutoClawJoint3,
            liftOdometry;
    public DigitalChannel liftReset, intakeDetect, foundationDetectLeft, foundationDetectRight;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public static String TAG = "MainThread";

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public AllHardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap, MecanumDrive drive) {
        if (!DriveConstants.VirtualizeDrive) {
            //region
            //------------------------===Drive Motors===------------------------
            backLeft = (DcMotorEx) hwMap.get(DcMotor.class, "leftRear");
            backRight = (DcMotorEx) hwMap.get(DcMotor.class, "rightRear");
            frontLeft = (DcMotorEx) hwMap.get(DcMotor.class, "leftFront");
            frontRight = (DcMotorEx) hwMap.get(DcMotor.class, "rightFront");
            //---------------------------------------------------------------------------
            //endregion

            gyro = hwMap.get(BNO055IMU.class, "imu");
            imu = (IntegratingGyroscope) gyro;

            this.hardwareMap = hwMap;
        }
        else  {
            frontLeft = new VirtualMotorEx(drive, "leftFront");
            backLeft = new VirtualMotorEx(drive, "leftRear");
            backRight = new VirtualMotorEx(drive, "rightRear");
            frontRight = new VirtualMotorEx(drive, "rightFront");

            this.hardwareMap = hwMap;
        }
    }


    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        gyro.initialize(parameters);
    }
}
