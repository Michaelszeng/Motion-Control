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
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    FieldDashboard fieldDashboard = new FieldDashboard(drive);
    public void runOpMode() throws InterruptedException {
        double s = drive.getPoseEstimate().getX();
        while (! isStopRequested()) {
            while(s < 60) {
                drive.setMotorPowers(0.8, 0.8, 0.8, 0.8);
                List<Double> wheelPositions = drive.getWheelPositions();
                RobotLogger.dd(TAG, "wheelPositions:" + wheelPositions.toString());
                s = drive.getPoseEstimate().getX();

                RobotLogger.dd(TAG, "localizer:" + drive.getLocalizer().getPoseEstimate().toString());
                fieldDashboard.updateDashboard();

                SafeSleep.sleep_milliseconds(this, 10);
                s = drive.getPoseEstimate().getX();
            }
        }
    }
}
