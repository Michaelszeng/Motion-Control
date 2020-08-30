package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to use virtual localizer with virtual drive train
 */
@Config
@Autonomous(group = "drive")
public class SimpleDrive2 extends LinearOpMode {
    private String TAG = "SimpleDrive2";

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        FieldDashboard fieldDashboard = new FieldDashboard(drive);
        Pose2d currentPose = drive.getPoseEstimate();
        double currentX = -currentPose.getY();
        double currentY = currentPose.getX();
        double currentHeading = currentPose.getHeading();

        waitForStart();

        while (opModeIsActive()) {
            //FL, BL, BR, FR
            drive.setMotorPowers(0.8, 0.8, 0.0, 0.0);

            List<Double> wheelPositions = drive.getWheelPositions();

            currentPose = drive.getPoseEstimate();
            currentX = -currentPose.getY();
            currentY = currentPose.getX();
            currentHeading = currentPose.getHeading();

            RobotLogger.dd(TAG, "localizer: (" + currentX + ", " + currentY + ", " + currentHeading + ")");
            fieldDashboard.updateDashboard();

            SafeSleep.sleep_milliseconds(this, 50);
        }
    }
}
