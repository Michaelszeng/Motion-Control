package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

/*
 * This is a simple routine to use virtual localizer with virtual drive train
 */
@Config
@Autonomous(group = "drive")
public class SimpleDrive4 extends LinearOpMode {
    private String TAG = "SimpleDrive4";
    List<Pose2d> poseHistory = new ArrayList<>();
    FtcDashboard dashboard;

    private List<Double> strafePowers = new ArrayList<>(Arrays.asList(0.3, -0.3, 0.3, -0.3));
    private List<Double> straightPowers = new ArrayList<>(Arrays.asList(0.3, 0.3, 0.3, 0.3));
    private List<Double> leftTurnPowers = new ArrayList<>(Arrays.asList(0.1, 0.1, 0.3, 0.3));
    double last_lf = 0, last_rf = 0, dRight = 0, dLeft = 0, rCenter = 0, lastX = 0, currentX = 0, lastY = 0, currentY = 0, lastHeading = 0, currentHeading = 0;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        double current_lf = 0;
        double current_rf = 0;
        double phi = 0, Px = 0, Py = 0;
        double EPSILON = 1e-6;

        while (!isStopRequested()) {
            drive.setMotorPowers(straightPowers.get(0), straightPowers.get(1), straightPowers.get(2), straightPowers.get(3));
            List<Double> wheelPositions = drive.getWheelPositions();
            current_lf = wheelPositions.get(1);
            current_rf = wheelPositions.get(2);
            RobotLogger.dd(TAG, "current front wheel positions: " + Double.toString(current_lf) + ", " + Double.toString(current_rf));
            //in cases where it's a straight line:
            dLeft = current_lf - last_lf;
            dRight = current_rf - last_rf;
            if (Math.abs(Math.abs(dRight) - Math.abs(dLeft)) <= EPSILON) {
                currentX = lastX + dRight * Math.cos(lastHeading);
                currentY = lastY + dRight * Math.sin(lastHeading);;
                RobotLogger.dd(TAG, "moving straight...TBD");
            } else {
                //finding phi:
                phi = (dRight - dLeft) / (TRACK_WIDTH*2);  // Mecanum wheel vs. Swerve wheel

                //applying phi:
                rCenter = ((dRight + dLeft) / 2) / phi;
                RobotLogger.dd(TAG, "phi: " + Double.toString(phi));
                RobotLogger.dd(TAG, "dRight: " + Double.toString(dRight));
                RobotLogger.dd(TAG, "dLeft: " + Double.toString(dLeft));
                RobotLogger.dd(TAG, "rCenter: " + Double.toString(rCenter));

                //for x':
                Px = lastX - rCenter * Math.sin(lastHeading);
                currentX = Px + rCenter * Math.cos(phi + lastHeading - Math.PI/2);

                //for y';
                Py = lastY + rCenter * Math.cos(lastHeading);
                currentY = Py + rCenter * Math.sin(phi + lastHeading - Math.PI/2);

                //for heading';
                currentHeading = lastHeading + phi;

                RobotLogger.dd(TAG, "Px: " + Double.toString(Px));
                RobotLogger.dd(TAG, "Py: " + Double.toString(Py));
                RobotLogger.dd(TAG, "rCenter: " + Double.toString(rCenter));


            }
            last_lf = current_lf;
            last_rf = current_rf;
            lastX = currentX;
            lastY = currentY;
            lastHeading = currentHeading;

            Pose2d RRpose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "Road Runner Pose " + RRpose.toString());
            Pose2d currentPose = new Pose2d(currentX, currentY, currentHeading);
            RobotLogger.dd(TAG, "My localizer pose " + currentPose.toString());

            Pose2d plotPose = currentPose;
            //for appearances:
            RobotLogger.dd(TAG, wheelPositions.toString());
            SafeSleep.sleep_milliseconds(this, 10);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            poseHistory.add(plotPose);
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
            DashboardUtil.drawRobot(fieldOverlay, plotPose);
            RobotLogger.dd(TAG, String.valueOf(currentX));
            packet.put("mode", SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY);

            packet.put("x", plotPose.getX());
            packet.put("y", plotPose.getY());
            packet.put("heading", plotPose.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
