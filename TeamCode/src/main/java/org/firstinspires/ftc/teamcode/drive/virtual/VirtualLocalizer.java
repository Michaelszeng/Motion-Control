package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.jetbrains.annotations.NotNull;

public class VirtualLocalizer implements Localizer {
    Pose2d _poseEstimate = new Pose2d(0, 0, 0);
    DriveTrain _driveTrain;
    private String TAG = "VirtualLocalizer";
    MecanumDrive _drive;
    public VirtualLocalizer(DriveTrain _virtualDriveTrain) {
        _drive = _virtualDriveTrain.drive;
        _driveTrain = _virtualDriveTrain;
        RobotLogger.dd(TAG, "VirtualLocalizer created");
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        update();
        return _poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        _poseEstimate = pose2d;
    }

    @Override
    public void update() {
        _poseEstimate = _driveTrain.getRobotPose();
        RobotLogger.dd(TAG, "pose updated " +  _poseEstimate.toString());
    }
    // Todo:
    @Override
    public Pose2d getPoseVelocity() {
        RobotLogger.callers(4, TAG, "getPoseVelocity for is not supported");
        return (new Pose2d(0, 0, 0));
    }
}
