package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.Date;

@TeleOp(name = "PIDTestVirtual3")
public class PIDTestVirtual3 extends LinearOpMode {
    private String TAG = "PIDTestVirtual3";

    Date datePrev = new Date();
    Date dateNew = new Date();
    double dateDiff;

    final double testDirection = Math.PI/4;     //0 degrees = West (Right)
    final double testFinalAngle = -Math.PI;
    //    final double testAngle = Math.PI/2;     //0 degrees = West (Right)
//    final double testAngle = Math.PI;     //0 degrees = West (Right)
    final double testDistance = 48; //look ahead distance
    double globalAngle;
    double xDistance;
    double yDistance;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot(hardwareMap, true,0.0, 0.0, Math.PI/2);
        ArrayList<Double> motorPowers;
        Pose2d currentTarget;

//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FieldDashboard fieldDashboard = new FieldDashboard(robot);

        Pose2d currentPose = robot.getPoseEstimate();
        double currentX = -currentPose.getY();
        double currentY = currentPose.getX();
        double currentHeading = currentPose.getHeading();

        waitForStart();

        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());
            RobotLogger.dd(TAG, String.valueOf(dateDiff));

            TelemetryPacket packet = new TelemetryPacket();

            xDistance = testDistance * Math.cos(testDirection);
            yDistance = testDistance * Math.sin(testDirection);

            currentTarget = new Pose2d(xDistance, yDistance, testFinalAngle);
            currentX = -robot.getPoseEstimate().getY();
            currentY = robot.getPoseEstimate().getX();
            currentHeading = normalizeHeading(robot.getPoseEstimate().getHeading());

            currentPose = new Pose2d(currentX, currentY, currentHeading);
            motorPowers = robot.update(currentPose, currentTarget, (int) dateDiff);
            //FL, BL, BR, FR
            robot.setMotorPowers(motorPowers.get(0), motorPowers.get(1), motorPowers.get(2), motorPowers.get(3));


            Log.d(TAG, "Loopcycle: " + dateDiff);
            RobotLogger.dd(TAG, "localizer: (" + currentX + ", " + currentY + ", " + currentHeading + ")");

            telemetry.update();

            fieldDashboard.updateDashboard();

            SafeSleep.sleep_milliseconds(this, 18);
            idle();
        }
    }

    public double normalizeHeading(double heading) {
        //Simulating behavior or real IMU--range of headings is +/- 180, North is 0
        double currentHeading = heading;
        while (currentHeading < -Math.PI) {
            currentHeading = currentHeading + (2 * Math.PI);
        }
        while (currentHeading > Math.PI) {
            currentHeading = currentHeading - (2 * Math.PI);
        }
        return currentHeading;
    }

    private void initHardwareMap(DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back) {
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