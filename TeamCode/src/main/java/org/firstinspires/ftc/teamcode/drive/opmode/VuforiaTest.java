package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.vision.VuforiaCameraChoice;


/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class VuforiaTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "VuforiaTest";
    private SampleMecanumDrive _drive = null;
    private HardwareMap hwMap;


    @Override
    public void runOpMode() throws InterruptedException {
        RobotLogger.dd(TAG, "unit test for vuforia localizer");

        waitForStart();

        if (isStopRequested()) return;
        VuforiaCamLocalizer vLocalizer = null;

        int count = 0;
        while (opModeIsActive()) {
            if (opModeIsActive())
                vLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_BACK, true);
            else
                break;
            FtcDashboard.getInstance().startCameraStream(vLocalizer.getVuforiaCameraSource(), 0);
            Pose2d poseEstimate;
            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive()) {
                    poseEstimate = vLocalizer.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                    RobotLogger.dd(TAG, "vuforia localization: " + poseEstimate.toString());
                }
                if (opModeIsActive()) {
                    SafeSleep.sleep_milliseconds(this,20000);
                }
            }
            vLocalizer.stop();

            count ++;
            if (opModeIsActive())
                vLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap, VuforiaCameraChoice.PHONE_FRONT, true);
            else
                break;
            for (int i = 0; i < 10; i ++ ) {
                if (opModeIsActive()) {
                    poseEstimate = vLocalizer.getPoseEstimate();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                    RobotLogger.dd(TAG, "vuforia localization: " + poseEstimate.toString());
                }
                if (opModeIsActive()) {
                    SafeSleep.sleep_milliseconds(this,20000);
                }
            }
            vLocalizer.stop();

            count ++;

        }
        FtcDashboard.getInstance().stopCameraStream();

        RobotLogger.dd(TAG, "----------done --------------------- unit test for vuforia localizer");
    }
}
