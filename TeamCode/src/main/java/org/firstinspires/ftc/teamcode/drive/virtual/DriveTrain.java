package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;

// motor to wheel
public class DriveTrain extends BaseDriveTrain {
    private String TAG = "DriveTrain";
    private static DriveTrain driveTrain_singleInstance = null;
    public DriveTrain(MecanumDrive _drv)
    {
        super(_drv);
        setPoseEstimate(new Pose2d(0, 0, 0));
        RobotLogger.dd("DriveTrain", "drive train created");
    }

    /*
    synchronized  public static DriveTrain getSingle_instance(MecanumDrive drv, String name) {
        if (driveTrain_singleInstance == null) {
            RobotLogger.dd("DriveTrain", "drive train created");
            driveTrain_singleInstance = new DriveTrain(drv);
            driveTrain_singleInstance.setPoseEstimate(new Pose2d(0, 0, 0));
        }

        return driveTrain_singleInstance;
    }
    */
    public Pose2d getRobotPose() {
        RobotLogger.dd(TAG, "getRobotHeading, num of motors: " + drive_motors.size());
        double extHeading = 0;
        List<Double> wheelPositions = drive.getWheelPositions();
        if (lastWheelPositions.size() != 0) {
            List<Double> wheelDeltas = new ArrayList<>();
            for (DcMotorEx motor : drive_motors) {
                int index = drive_motors.indexOf(motor);
                wheelDeltas.add(wheelPositions.get(index) - lastWheelPositions.get(index));
            }
            Pose2d robotPoseDelta = ForwardKinematics.wheelToRobotVelocities(wheelDeltas);
            double finalHeadingDelta = robotPoseDelta.getHeading();
            poseEstimate = ForwardKinematics.relativeOdometryUpdate(poseEstimate,
                    new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        List<Double> wheelVelocities = drive.getWheelVelocities();
        //double extHeadingVel = drive.getExternalHeadingVelocity()
        if (wheelVelocities != null) {
            Pose2d poseVelocity = ForwardKinematics.wheelToRobotVelocities(
                    wheelVelocities);

        }
        lastWheelVelocities = wheelVelocities;
        lastWheelPositions = wheelPositions;
        lastExtHeading = extHeading;
        return poseEstimate;
    }

    public void finalize() throws Throwable{
        RobotLogger.callers(8, TAG, "finalize");
        driveTrain_singleInstance = null;
    }
}
