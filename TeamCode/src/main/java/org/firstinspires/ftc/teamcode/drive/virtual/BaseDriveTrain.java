package org.firstinspires.ftc.teamcode.drive.virtual;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;

public abstract class BaseDriveTrain {
    protected List<DcMotorEx> drive_motors;
    List<Double> lastWheelVelocities, lastWheelPositions;
    double lastExtHeading;
    MecanumDrive drive;
    protected List<DcMotorEx> odom_motors;
    protected DcMotor leftOdomEncoder, rightOdomEncoder, frontOdomEncoder;
    boolean usingOdom;
    boolean usingIMU;
    private String TAG = "BaseDriveTrain";

    private static final int DRIVE_WHEEL_NUM = 4;
    private boolean driveTrainReady = false;
    protected Pose2d poseEstimate;

    public BaseDriveTrain(MecanumDrive _drive)
    {
        drive = _drive;
        drive_motors = new ArrayList<>();
        lastWheelVelocities = new ArrayList<>();
        lastWheelPositions = new ArrayList<>();
    }

    public int getWheelCount()
    {
        return drive_motors.size();
    }
    private int getMotorIndexFromName(String name) {
        int index = -1;
        if (name.equalsIgnoreCase("leftFront"))
            index = 0;
        else if (name.equalsIgnoreCase("leftRear"))
            index = 1;
        else if (name.equalsIgnoreCase("rightRear"))
            index = 2;
        else if (name.equalsIgnoreCase("rightFront"))
            index = 3;
        else
            RobotLogger.dd(TAG, "unexpected motor");
        return index;
    }
    private String getMotorNameFromIndex(int index) {
        if (index == 0)
            return "leftFront";
        else if (index == 1)
            return "leftRear";
        else if (index == 2)
            return "rightRear";
        else if (index == 3)
            return "rightFront";
        else
            RobotLogger.dd(TAG, "unexpected motor");
        return null;
    }
    private void clear() {
        drive_motors.clear();
        lastWheelVelocities.clear();
        lastWheelPositions.clear();
    }
    public void AddMotors(List<DcMotorEx> motors) {
        for (int i = 0; i < motors.size(); i ++) {
            AddWheel(motors.get(i), getMotorNameFromIndex(i));
        }
    }
    public void AddWheel(DcMotorEx motor, String name) {
        RobotLogger.dd(TAG, "add wheel: " + name);
        if (getWheelCount() == DRIVE_WHEEL_NUM && name.equals("leftFront"))
        {
            RobotLogger.dd(TAG, "need to reset motors, SHOULD NOT HAPPEN!");
            clear();
            driveTrainReady = false;
        }
        drive_motors.add(motor);
        if (getWheelCount() == DRIVE_WHEEL_NUM)
        {
            poseEstimate = new Pose2d(0, 0, 0);
            driveTrainReady = true;
            RobotLogger.dd(TAG, "4 wheels are ready");
        }
    }
    public void setPoseEstimate(Pose2d pose)
    {
        poseEstimate = pose;
    }
    abstract public Pose2d getRobotPose() ;
}
