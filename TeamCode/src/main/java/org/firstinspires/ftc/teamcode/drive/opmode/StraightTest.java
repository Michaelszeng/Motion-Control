package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class StraightTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        while (! isStopRequested()) {
            Trajectory trajectory = drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate())
                    .forward(DISTANCE)
                    .build();

            waitForStart();

            if (isStopRequested()) return;

            SafeSleep.sleep_milliseconds(this, 500);

            drive.followTrajectory(trajectory);

            trajectory = drive.trajectoryBuilder(drive.getLocalizer().getPoseEstimate())
                    .back(DISTANCE)
                    .build();
            drive.followTrajectory(trajectory);
            SafeSleep.sleep_milliseconds(this,500);
        }
    }
}
