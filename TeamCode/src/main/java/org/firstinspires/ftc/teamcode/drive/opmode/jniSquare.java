package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class jniSquare extends LinearOpMode {
    public native String  stringFromJNI();
    public native int  squared(int num);
    static{
        System.loadLibrary("hello-jni");
    }

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        long start = System.currentTimeMillis();
        while (opModeIsActive()) {
            int time = (int)(System.currentTimeMillis() - start)/1000;
            telemetry.addData("count", time);
            telemetry.addData("squared", squared(time));
            telemetry.update();
        }
    }
}