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
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_TRACK_WIDTH;
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

    double last_lf = 0, last_rf = 0, dRight = 0, dLeft = 0, rCenter = 0, lastX = 0, currentX = 0, lastY = 0, currentY = 0, lastHeading = 0, currentHeading = 0;

    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        double current_lf = 0;
        double current_rf = 0;
        double phi = 0;
        double Px = 0, Py = 0;
        double EPSILON = 1e-6;

        while (!isStopRequested()) {
            drive.setMotorPowers(0.3, 0.3, 0.2, 0.2);
            List<Double> wheelPositions = drive.getWheelPositions();
            current_lf = wheelPositions.get(0);
            current_rf = wheelPositions.get(3);

            //in cases where it's a straight line:
            dRight = current_lf - last_lf;
            dLeft = current_rf - last_rf;
            if (Math.abs(Math.abs(dRight) - Math.abs(dLeft)) <= EPSILON) {
                //currentX = ;
                //currentY = ;
                RobotLogger.dd(TAG, "moving straight...TBD");
            } else {
                last_lf = current_lf;
                last_rf = current_rf;

                //finding phi:
                phi = (dRight - dLeft) / ODOMETRY_TRACK_WIDTH;

                //applying phi:
                rCenter = ((dRight + dLeft) / 2) / phi;
                RobotLogger.dd(TAG, "phi: " + Double.toString(phi));
                RobotLogger.dd(TAG, "dRight: " + Double.toString(dRight));
                RobotLogger.dd(TAG, "dLeft: " + Double.toString(dLeft));
                RobotLogger.dd(TAG, "rCenter: " + Double.toString(rCenter));

                //for x':
                Px = lastX - rCenter * Math.sin(lastHeading);
                currentX = Px + rCenter * Math.cos(phi + lastHeading - 90);

                //for y';
                Py = lastY + rCenter * Math.cos(lastHeading);
                currentY = Py + rCenter * Math.sin(phi + lastHeading - 90);

                currentHeading = lastHeading + phi;

                RobotLogger.dd(TAG, "Px: " + Double.toString(Px));
                RobotLogger.dd(TAG, "Py: " + Double.toString(Py));
                RobotLogger.dd(TAG, "rCenter: " + Double.toString(rCenter));

                //for heading';
                lastX = currentX;
                lastY = currentY;
                lastHeading = currentHeading;
            }
            last_lf = current_lf;
            last_rf = current_rf;

            Pose2d RRpose = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "Road Runner Pose " + RRpose.toString());
            Pose2d currentPose = new Pose2d(currentX, currentY, currentHeading);
            RobotLogger.dd(TAG, "My localizer pose " + currentPose.toString());

            //for appearances:
            RobotLogger.dd(TAG, wheelPositions.toString());
            SafeSleep.sleep_milliseconds(this, 10);

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            poseHistory.add(currentPose);
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.setStroke("#3F51B5");
            DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
            DashboardUtil.drawRobot(fieldOverlay, currentPose);
            RobotLogger.dd(TAG, String.valueOf(currentX));
            packet.put("mode", SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY);

            packet.put("x", currentPose.getX());
            packet.put("y", currentPose.getY());
            packet.put("heading", currentPose.getHeading());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
