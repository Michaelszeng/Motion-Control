package org.firstinspires.ftc.teamcode.drive;

import android.support.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.virtual.DriveTrain;
import org.firstinspires.ftc.teamcode.drive.virtual.VirtualDeadMotor;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_FORWARD_OFFSET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.ODOMETRY_TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.VirtualizeDrive;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.odoEncoderTicksPerRev;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = odoEncoderTicksPerRev;
    public static double WHEEL_RADIUS = 1.25; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = ODOMETRY_TRACK_WIDTH; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = ODOMETRY_FORWARD_OFFSET; // in; offset of the lateral wheel

    private String TAG = "StandardTrackingWheelLocalizer";

    private DcMotor leftEncoder, rightEncoder, frontEncoder;
    private DriveTrain driveTrain;
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));
        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        frontEncoder = hardwareMap.dcMotor.get("frontEncoder");
    }
    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap, DriveTrain _drive) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));
        RobotLogger.dd(TAG, "StandardTrackingWheelLocalizer created for virtual odometry wheels");

        driveTrain = _drive;
        VirtualDeadMotor.resetDeadMotors();
        leftEncoder = new VirtualDeadMotor(driveTrain, "leftEncoder");
        rightEncoder = new VirtualDeadMotor(driveTrain, "rightEncoder");
        frontEncoder = new VirtualDeadMotor(driveTrain, "frontEncoder");
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }
    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        RobotLogger.dd(TAG, "getPoseEstimate");
        update();
        return super.getPoseEstimate();
    }
    @Override
    public void update() {
        super.update();
        RobotLogger.dd(TAG, "pose updated " +  super.getPoseEstimate().toString());
    }
    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        RobotLogger.dd(TAG, "getWheelPositions");
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }
}
