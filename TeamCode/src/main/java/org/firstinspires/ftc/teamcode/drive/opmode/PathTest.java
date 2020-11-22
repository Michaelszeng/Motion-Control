package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutoPath;
import org.firstinspires.ftc.teamcode.autonomous.FieldPosition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.vision.VuforiaCamLocalizer;
import org.firstinspires.ftc.teamcode.vision.VuforiaCameraChoice;


@Config
@Autonomous(group = "drive")
//@Disabled
public class PathTest extends LinearOpMode {
    private Trajectory trajectory;
    private BaseTrajectoryBuilder builder, strafe_builder;
    private Pose2d current_pose;
    private String TAG = "PathTest";
    private SampleMecanumDrive _drive = null, _strafeDrive = null;
    private AllHardwareMap hwMap;
    private AutoPath path;
    private FieldPosition fieldPosition = null;
    private VuforiaCamLocalizer vuLocalizer = null;

    @Override
    public void runOpMode() throws InterruptedException {
        // do we need this??? all this are HW settings to enable ARM actions;
        //hwMap.liftOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //hwMap.liftOne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //hwMap.liftOne.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        int[] skystonePositions = new int[2];
        skystonePositions[0] = (int) DriveConstants.TEST_SKY_STONE_POSITION;
        RobotLogger.dd(TAG, "xml file %d", skystonePositions[0]);
        String filename = "path_blue_.xml";
        int tindex = filename.indexOf(".xml");
        RobotLogger.dd(TAG, "%d", tindex);
        filename = filename.substring(0, tindex-1) + Integer.toString(skystonePositions[0])
                + filename.substring(tindex);
        Pose2d t[] = DriveConstants.parsePathXY(filename);
        RobotLogger.dd(TAG, "XY array len: " + Integer.toString(t.length));

        _drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startingPos = new Pose2d(new Vector2d(-34.752, -63.936), Math.toRadians(0));
        hwMap = new AllHardwareMap(hardwareMap, _drive);
        fieldPosition = FieldPosition.RED_QUARY;

        waitForStart();

        if (isStopRequested()) return;

        if (DriveConstants.USE_VUFORIA_LOCALIZER) {
            vuLocalizer = VuforiaCamLocalizer.getSingle_instance(hardwareMap,
                    VuforiaCameraChoice.PHONE_BACK, true);
        }
        path = new AutoPath(hwMap, this, _drive, hardwareMap, null, telemetry);
//        path.BlueQuary(skystonePositions, vuLocalizer);
        RobotLogger.dd(TAG, "----------done --------------------- unit test for path (BLUE QUARY)");
    }
}
