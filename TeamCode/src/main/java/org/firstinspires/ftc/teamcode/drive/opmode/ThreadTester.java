//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import android.content.Context;
//import android.hardware.usb.UsbDeviceConnection;
//import android.hardware.usb.UsbManager;
//
//import
//
//import com.hoho.android.usbserial.driver.UsbSerialDriver;
//import com.hoho.android.usbserial.driver.UsbSerialPort;
//import com.hoho.android.usbserial.driver.UsbSerialProber;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.util.RobotLogger;
//import org.firstinspires.ftc.teamcode.util.SafeSleep;
//
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.Date;
//import java.util.List;
//
//@TeleOp(name = "ThreadTester")
//public class ThreadTester extends LinearOpMode {
//    private String TAG = "ThreadTester";
//
//    Date datePrev = new Date();
//    Date dateNew = new Date();
//    double dateDiff;
//
//    int loop = 0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//        waitForStart();
//
//        while (opModeIsActive()) {   //put teleop code in here
//            dateNew.setTime(new Date().getTime());
//            dateDiff = dateNew.getTime() - datePrev.getTime();
//            datePrev.setTime(dateNew.getTime());
//            RobotLogger.dd(TAG, String.valueOf(dateDiff));
//
//            RobotLogger.dd(TAG, "Loop: " + loop + "\tLoop-time: " + dateDiff);
//            loop += 1;
//
//            telemetry.update();
//
//            SafeSleep.sleep_milliseconds(this, 18);
//            idle();
//        }
//    }
//
//    public void openSerialDevice() throws IOException {
//        // Find all available drivers from attached devices.
//        UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
//        List<UsbSerialDriver> availableDrivers = UsbSerialProber.getDefaultProber().findAllDrivers(manager);
//        if (availableDrivers.isEmpty()) {
//            return;
//        }
//
//        // Open a connection to the first available driver.
//        UsbSerialDriver driver = availableDrivers.get(0);
//        UsbDeviceConnection connection = manager.openDevice(driver.getDevice());
//        if (connection == null) {
//            // add UsbManager.requestPermission(driver.getDevice(), ..) handling here
//            return;
//        }
//
//        UsbSerialPort port = driver.getPorts().get(0); // Most devices have just one port (port 0)
//        port.open(connection);
//        port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE);
//    }
//}
