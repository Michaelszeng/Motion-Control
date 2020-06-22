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
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to test simple drive capabilities with setPower and getWheelPositions
 */
@Config
@Autonomous(group = "drive")
public class SimpleDrive extends LinearOpMode {
    private String TAG = "SimpleDrive";

    List<Pose2d> poseHistory = new ArrayList<>();
    FtcDashboard dashboard;
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        double s = 0;
        while (! isStopRequested()) {
            while(s < 60) {
                //FL, BL, BR, FR
                drive.setMotorPowers(0.8,0.0, 0.0, 0.0);
                List<Double> wheelPositions = drive.getWheelPositions();
                s = wheelPositions.get(0);
                RobotLogger.dd(TAG, wheelPositions.toString());
                SafeSleep.sleep_milliseconds(this, 10);

                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();
                Pose2d currentPose = new Pose2d(s, 0, 0);
                poseHistory.add(currentPose);
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);
                RobotLogger.dd(TAG, String.valueOf(s));
                packet.put("mode", SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY);

                packet.put("x", currentPose.getX());
                packet.put("y", currentPose.getY());
                packet.put("heading", currentPose.getHeading());
                dashboard.sendTelemetryPacket(packet);

            }
        }
    }
}
