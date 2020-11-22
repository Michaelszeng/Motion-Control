package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;

public class FieldDashboard {
    FtcDashboard _dashBoard;
    MecanumDrive _drive;
    private String TAG = "FieldDashboard";
    List<Pose2d> _poseHistory;
    public FieldDashboard(MecanumDrive drive) {
        _drive = drive;
        _dashBoard = FtcDashboard.getInstance();
        _dashBoard.setTelemetryTransmissionInterval(25);
        _poseHistory = new ArrayList<>();
    }

    public void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();
        Pose2d currentPose = _drive.getPoseEstimate();
        _poseHistory.add(currentPose);
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#3F51B5");
        DashboardUtil.drawPoseHistory(fieldOverlay, _poseHistory);
        DashboardUtil.drawRobot(fieldOverlay, currentPose);
        packet.put("mode", SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY);

        packet.put("x", currentPose.getX());
        packet.put("y", currentPose.getY());
        packet.put("heading", currentPose.getHeading());
        _dashBoard.sendTelemetryPacket(packet);
        return;
    }
}
