package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

public class AllHardwareMap {
    //"adb connect 192.168.43.1:5555"
    public DcMotorEx backLeft, backRight, frontLeft, frontRight;
    public BNO055IMU gyro;
    public IntegratingGyroscope imu;
    public static String TAG = "MainThread";

    public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;

    public AllHardwareMap(com.qualcomm.robotcore.hardware.HardwareMap hwMap) {

        //region
        //------------------------===Drive Motors===------------------------
        backLeft = (DcMotorEx) hwMap.get(DcMotor.class, "backLeft");
        backRight = (DcMotorEx) hwMap.get(DcMotor.class, "backRight");
        frontLeft = (DcMotorEx) hwMap.get(DcMotor.class, "frontLeft");
        frontRight = (DcMotorEx) hwMap.get(DcMotor.class, "frontRight");
        //---------------------------------------------------------------------------
        //endregion

        gyro = hwMap.get(BNO055IMU.class, "imu");
        imu = (IntegratingGyroscope) gyro;

        this.hardwareMap = hwMap;
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        gyro.initialize(parameters);
    }
}
