package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in
    public String TAG = "StraightTest";
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while (! isStopRequested()) {
            Pose2d currentPos = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "current position: " + currentPos);
            Trajectory trajectory = drive.trajectoryBuilder(currentPos)
                    .forward(DISTANCE)
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            drive.followTrajectory(trajectory);
            SafeSleep.sleep_milliseconds(this, 500);

            currentPos = drive.getPoseEstimate();
            RobotLogger.dd(TAG, "current position: " + currentPos);

            trajectory = drive.trajectoryBuilder(currentPos)
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(trajectory);
            SafeSleep.sleep_milliseconds(this,500);
        }
    }
}
