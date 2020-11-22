package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to do straight/strafe move with localizer and setPower.
 */
@Config
@Autonomous(group = "drive")
public class SimpleDrive3 extends LinearOpMode {
    public double strafeTimeDistanceRatio = 0.093; // duration for power to achieve strafe distance;
    //public double strafeMotorPower = 0.19;
    public double rear_ratio = 1.105;
    public int pollingInterval = 10; // in milliseconds;
    SampleMecanumDrive drive;
    private String TAG = "SimpleDrive3";

    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        double s = 0;
        if (! isStopRequested()) {
            manualStrafe(0.5, 24, false);
            manualStraight(0.5, 48, false);
        }
    }
    public void manualStraight(double straightPower, double distance, boolean backward) {
        FieldDashboard fieldDashboard = new FieldDashboard(drive);
        long duration = (long) (1000 * distance * strafeTimeDistanceRatio);
        RobotLog.dd(TAG, "set power for move straight: " + Double.toString(straightPower) + " duration: " + Double.toString(duration));

        Pose2d start_pose = drive.getPoseEstimate();

        boolean done = false;
        while((! done) && (! isStopRequested())) {
            if (backward) {
                drive.setMotorPowers(-1 * straightPower, -1 * straightPower * rear_ratio, -1 * straightPower * rear_ratio, -1 * straightPower);
            } else {
                drive.setMotorPowers(straightPower, straightPower * rear_ratio, straightPower * rear_ratio, straightPower);
            }

            Pose2d pose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, pose.toString());
            fieldDashboard.updateDashboard();

            if (Math.abs(pose.getX() - start_pose.getX()) >= distance)
                done = true;
            SafeSleep.sleep_milliseconds(this, pollingInterval);
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }
    public void manualStrafe(double strafeMotorPower, double distance, boolean left) {
        FieldDashboard fieldDashboard = new FieldDashboard(drive);
        long duration = (long) (1000 * distance * strafeTimeDistanceRatio);
        RobotLog.dd(TAG, "set power for strafe: " + Double.toString(strafeMotorPower) + " duration: " + Double.toString(duration));

        Pose2d start_pose = drive.getPoseEstimate();

        boolean done = false;
        while((! done) && (! isStopRequested())) {
            if (left) {
                drive.setMotorPowers(-1 * strafeMotorPower, strafeMotorPower * rear_ratio, -1 * strafeMotorPower * rear_ratio, strafeMotorPower);
            } else {
                drive.setMotorPowers(strafeMotorPower, -1 * strafeMotorPower * rear_ratio, strafeMotorPower * rear_ratio, -1 * strafeMotorPower);
            }

            Pose2d pose = drive.getLocalizer().getPoseEstimate();
            RobotLogger.dd(TAG, pose.toString());
            fieldDashboard.updateDashboard();

            if (Math.abs(pose.getY() - start_pose.getY()) >= distance)
                done = true;
            SafeSleep.sleep_milliseconds(this, pollingInterval);
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }
}
