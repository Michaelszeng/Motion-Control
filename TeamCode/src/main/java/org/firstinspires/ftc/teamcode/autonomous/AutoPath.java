package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.BaseTrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Teleop.TeleopConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;
import org.firstinspires.ftc.teamcode.vision.VuforiaCamLocalizer;

import java.util.List;

import static java.lang.Math.PI;

public class AutoPath {
    private Pose2d startingPos;
    private SampleMecanumDrive _drive; // default drive;
    private int step_count = 0;
    private BaseTrajectoryBuilder builder;
    private Trajectory trajectory;
    //private Align align;
    private AllHardwareMap hwMap;
    private LinearOpMode opMode;
    private List<Recognition> tfod;
    private com.qualcomm.robotcore.hardware.HardwareMap hardwareMap;
    private static String TAG = "AutonomousPath";
    private Pose2d currentPos;
    private Telemetry telemetry;
    private String path_file;
    private int first_skystone_location = 0;

    public AutoPath(AllHardwareMap hwMap, LinearOpMode opMode, SampleMecanumDrive drive,
                    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap, BNO055IMU imu, Telemetry telemetry) {
        //this.startingPos = startingPos;
        this.hwMap = hwMap;
        this.opMode = opMode;
        this.hardwareMap = hardwareMap;
        //align = new Align(hwMap, opMode, DcMotor.ZeroPowerBehavior.BRAKE);

        this._drive = drive;
        this.telemetry = telemetry;
        //vu = new VuforiaCamLocalizer(hardwareMap);
    }

    public static void StrafeDiagonalHelper(SampleMecanumDrive _drive, Vector2d dest) {
        Trajectory trajectory;
        Pose2d currentPos = _drive.getPoseEstimate();
        TrajectoryBuilder builder = null;
        builder = new TrajectoryBuilder(currentPos, DriveConstants.BASE_CONSTRAINTS);

        Pose2d error_pose = _drive.follower.getLastError();

        double current_x = currentPos.getX();
        double current_y = currentPos.getY();
        double delta_x = dest.getX() - current_x;
        double delta_y = dest.getY() - current_y;

        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, xy: %s", dest.toString());
        Vector2d firstStop;
        if (Math.abs(delta_x) > Math.abs(delta_y)) {
            RobotLogger.dd(TAG, "x > y, line first and then strafe");
            double square_offset = Math.abs(delta_y);
            double new_x = 0;
            if (delta_x > 0)
                new_x = dest.getX() - square_offset;
            else
                new_x = dest.getX() + square_offset;

            firstStop = new Vector2d(new_x, current_y);
            RobotLogger.dd(TAG, "added one line to stop: " + firstStop.toString());
            boolean dir = (firstStop.getX() - current_x) < 0 ? true : false;
            builder.lineTo(firstStop).strafeTo(dest);
        } else if (Math.abs(delta_x) < Math.abs(delta_y)) {
            RobotLogger.dd(TAG, "x < y, strafe first and then line");
            double square_offset = Math.abs(delta_x);
            double new_y = 0;
            if (delta_y < 0)
                new_y = current_y - square_offset;
            else
                new_y = current_y + square_offset;

            firstStop = new Vector2d(dest.getX(), new_y);
            RobotLogger.dd(TAG, "added one strafe stop: " + firstStop.toString());
            builder.strafeTo(firstStop).strafeTo(dest);
        } else {
            //double y_offset = delta_y - delta_x;
            builder.strafeTo(dest);
        }
        trajectory = builder.build();   //x - 2.812, y + 7.984
        _drive.followTrajectory(trajectory);

        currentPos = _drive.getPoseEstimate();
        error_pose = _drive.follower.getLastError();
        RobotLogger.dd(TAG, "StrafeDiagonalHelper, currentPos %s, errorPos %s", currentPos.toString(), error_pose.toString());
    }

    /*
    input: last pose from previous move;
    return: drive instance;
     */
    private SampleMecanumDrive DriveBuilderReset(boolean isStrafe, boolean init_imu, String label) {
            currentPos = _drive.getPoseEstimate();
            Pose2d newPos = currentPos;
            Pose2d error_pose = _drive.follower.getLastError();
            RobotLogger.dd(TAG, "start new step: %s, count[%d], currentPos %s, errorPos %s",
                    label, step_count, currentPos.toString(), error_pose.toString());
            if (DriveConstants.ENABLE_ARM_ACTIONS == false) {
                SafeSleep.sleep_milliseconds(opMode, (int) DriveConstants.TEST_PAUSE_TIME);
            }

            _drive.getLocalizer().setPoseEstimate(currentPos);
            _drive.getLocalizer().update();
            builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
            RobotLogger.dd(TAG, "drive and builder created, initialized with pose: " + _drive.getPoseEstimate().toString());
            return _drive;
        }

    public static void initGrab(AllHardwareMap hw, FieldPosition side, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
        mode) {
        transferReset(hw, mode);
        initIntakeClaw(hw, mode);
        init(hw, mode);
        prepGrab(hw, side, true, mode);    //*******
    }

    private int FollowPathFromXMLFile(Pose2d coordinates[], VuforiaCamLocalizer vLocal, boolean isRed) {
        Pose2d error_pose;
        int xy_len = coordinates.length;
        if (xy_len == 0) {
            telemetry.addData("read path XY failure: ", path_file);
            telemetry.update();
            RobotLogger.dd(TAG, "failed to read xml file");
            return -1;
        }
        RobotLogger.dd(TAG, "finished reading path coordinates num: " + Integer.toString(xy_len));
        step_count = 0;

        startingPos = coordinates[step_count];
        RobotLogger.dd(TAG, "step" + Integer.toString(step_count) + coordinates[step_count].toString());
        step_count++;

        _drive.setPoseEstimate(startingPos);
        //_drive.getLocalizer().setPoseEstimate(startingPos);
        _drive.update();

        double theta;

        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            initGrab(hwMap, isRed ? FieldPosition.RED_QUARY : FieldPosition.BLUE_QUARY, opMode);
        }
        // step 1;

        DriveBuilderReset(DriveConstants.USING_STRAFE_DIAGONAL, false,
                "step" + Integer.toString(step_count) + coordinates[step_count].toString() + ", after prepare, start");
        if (first_skystone_location == 1) {
            builder = builder
                    .strafeTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
            trajectory = builder.build();   //x - 2.812, y + 7.984
            if (opMode.opModeIsActive())
                _drive.followTrajectory(trajectory);
            else
                return -1;
        } else {
            if (opMode.opModeIsActive())
                AutoPath.StrafeDiagonalHelper(_drive, new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
            else
                return -1;
        }
        step_count++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before grab stone! Vuforia local info: " + t.toString());
        }
        RobotLogger.dd(TAG, "step1.5, after strafe, to grab");
        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            if (isRed)
                grabStone(hwMap, FieldPosition.RED_QUARY, opMode);   //*******
            else
                grabStone(hwMap, FieldPosition.BLUE_QUARY, opMode);
        }

        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            if (isRed)
                dropStone(hwMap, FieldPosition.RED_QUARY, true, opMode); //*******
            else
                dropStone(hwMap, FieldPosition.BLUE_QUARY, true, opMode);
        }

        // step 2;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after grab , to go straight");

        builder = builder
                .lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        else
            return -1;
        step_count++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before drop stone! Vuforia local info: " + t.toString());
        }

        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            FieldPosition fp = isRed ? FieldPosition.RED_QUARY : FieldPosition.BLUE_QUARY;
            prepGrab(hwMap, fp, false, opMode); //*******
        }

        RobotLogger.dd(TAG, "step2.5, after straight");


        // step 3;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop 1st stone, to straight move back");

        builder = builder
                .lineTo((new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY())));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        step_count++;
        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before grab 2nd stone! Vuforia local info: " + t.toString());
        }

        SafeSleep.sleep_milliseconds(opMode, 100);

        if (DriveConstants.ENABLE_ARM_ACTIONS) {
            if (isRed)
                dropStone(hwMap, FieldPosition.RED_QUARY, false, opMode);   //*******
            else
                dropStone(hwMap, FieldPosition.BLUE_QUARY, false, opMode);
        }

        // step 4;
        error_pose = _drive.follower.getLastError();
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                "after straight move, grabbed 2nd, to straight move");
        builder = builder
                .lineTo(new Vector2d(coordinates[step_count].getX(), coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        step_count++;

        if (vLocal != null) {
            Pose2d t = vLocal.getPoseEstimate();
            RobotLogger.dd(TAG, "Calibrate before drop 2nd stone! Vuforia local info: " + t.toString());
        }
        RobotLogger.dd(TAG, "step4.5, after straight move, to drop");

        SafeSleep.sleep_milliseconds(opMode, 400);

        // step 5
        DriveBuilderReset(true, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop 2nd stone, to strafe");
        builder = builder
                .strafeTo(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        step_count++;

        /*if (opMode.opModeIsActive()) {
            if (isRed) {
                hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
                sleep_millisec(200);
                hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);
                sleep_millisec(200);

                hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted);
                sleep_millisec(200);
            } else {
                hwMap.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init_blue);
                sleep_millisec(200);

                hwMap.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init_blue);
                sleep_millisec(200);

                hwMap.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted_blue);
                sleep_millisec(200);

            }
        }*/

        // step 6
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop and strafe");

        if (opMode.opModeIsActive()) {
            if (!isRed) {
                theta = _drive.getExternalHeading() >= 0 ? _drive.getExternalHeading() :
                        _drive.getExternalHeading() + 2 * PI;

                if (theta > PI)
                    _drive.turn(2 * PI - (_drive.getExternalHeading() - PI / 2));
                else
                    _drive.turn(-(_drive.getExternalHeading() - PI / 2));
            } else {
                theta = _drive.getExternalHeading() >= 0 ? _drive.getExternalHeading() :
                        _drive.getExternalHeading() + 2 * PI;

                if (theta > PI)
                    _drive.turn(-(_drive.getExternalHeading() - 3 * PI / 2));
                else
                    _drive.turn(-(_drive.getExternalHeading() + 2 * PI - 3 * PI / 2));
            }
        }

        // step 6
        DriveBuilderReset(false, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after foundation unlock, to straight move closer to foundation");
        builder = builder.lineTo(new Vector2d(_drive.getPoseEstimate().getX(),
                _drive.getPoseEstimate().getY() + coordinates[step_count].getY()));

        //builder = builder.setReversed(true).lineTo(new Vector2d(coordinates[step_count].getX(),
        //        coordinates[step_count].getY()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        step_count++;

        // step 7
        if (DriveConstants.ENABLE_ARM_ACTIONS) {

            hwMap.foundationLock.setPosition(TeleopConstants.foundationLockLock);
            hwMap.transferLock.setPosition(TeleopConstants.transferLockPosUp);
        }
        SafeSleep.sleep_milliseconds(opMode, 400);
        DriveBuilderReset(true, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", after drop fundation,, to spline ");
        builder.splineTo(new Pose2d(new Vector2d(_drive.getPoseEstimate().getX() - coordinates[step_count].getX(),
                _drive.getPoseEstimate().getY() - coordinates[step_count].getY()), coordinates[step_count].getHeading()));
        /*
        builder = builder.setReversed(false)
                .splineTo(new Pose2d(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()), coordinates[step_count].getHeading()));
         */
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        else
            return -1;
        step_count++;

        // step 8

        if(DriveConstants.ENABLE_ARM_ACTIONS){
            hwMap.foundationLock.setPosition(TeleopConstants.foundationLockUnlock);
            hwMap.transferLock.setPosition(TeleopConstants.transferLockPosOut);
        }

        //SafeSleep.sleep_milliseconds(300, opMode);

        DriveBuilderReset(true, false, "step" + Integer.toString(step_count) + coordinates[step_count].toString() +
                ", spline, back to parking");
        //builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
        builder = builder
                .splineTo(new Pose2d(new Vector2d(coordinates[step_count].getX(),
                        coordinates[step_count].getY()), coordinates[step_count].getHeading()));
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
        else
            return -1;
        step_count++;

        /*while (opMode.opModeIsActive()) {
            String o = "";
            for (Pose2d p2d : coordinates)
                o += p2d.toString() + ", ";
            telemetry.addData("Pose2d", o);
            telemetry.update();
        }*/
        return 0;
    }

    public void RedQuary(int[] skystonePositions, VuforiaCamLocalizer vuLocalizer) {
        String tmp = "path_red_.xml";
        first_skystone_location = skystonePositions[0];
        if (first_skystone_location > 3) {
            first_skystone_location = 3;
            RobotLogger.dd(TAG, "detected wrong, force skystone location to be [3, 6]");
        }
        else if (first_skystone_location < 1) {
            first_skystone_location = 1;
            RobotLogger.dd(TAG, "detected wrong, force skystone location to be [1, 4]");
        }
        path_file = tmp.substring(0, 8) + Integer.toString(skystonePositions[0])
                + tmp.substring(9);
        RobotLogger.dd(TAG, "to read XY coordinates from " + path_file);

        Pose2d xys1[] = DriveConstants.parsePathXY(path_file);
        if (opMode.opModeIsActive())
            FollowPathFromXMLFile(xys1, vuLocalizer, true);
    }

        public void RedFoundationPark() {
        hwMap.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset(hwMap, opMode);
        initIntakeClaw(hwMap, opMode);
        SafeSleep.sleep_milliseconds(opMode, 5000);

        hwMap.frontRight.setPower(0.5);
        hwMap.frontLeft.setPower(0.5);
        hwMap.backRight.setPower(0.7);
        hwMap.backLeft.setPower(0.7);

        SafeSleep.sleep_milliseconds(opMode, 400);

        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);

       /* DriveBuilderReset(false, false, "step");
        builder = builder.forward(24);
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectorySync(trajectory);
        intake(hwMap, 0);*/
    }

    public void BlueQuary(int[] skystonePositions, VuforiaCamLocalizer vuLocalizer) {    // (-x, y)
        String tmp = "path_blue_.xml";
        first_skystone_location = skystonePositions[0];
        if (first_skystone_location > 3) {
            first_skystone_location = 3;
            RobotLogger.dd(TAG, "detected wrong, force skystone location to be [3, 6]");
        }
        else if (first_skystone_location < 1) {
            first_skystone_location = 1;
            RobotLogger.dd(TAG, "detected wrong, force skystone location to be [1, 4]");
        }
        path_file = tmp.substring(0, 9) + Integer.toString(skystonePositions[0])
                + tmp.substring(10);
        RobotLogger.dd(TAG, "to read XY coordinates from " + path_file);

        Pose2d xys1[] = DriveConstants.parsePathXY(path_file);
        if (opMode.opModeIsActive())
            FollowPathFromXMLFile(xys1, vuLocalizer, false);
    }

        public void BlueFoundationPark() {
        hwMap.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hwMap.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hwMap.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
        transferReset(hwMap, opMode);
        initIntakeClaw(hwMap, opMode);
        SafeSleep.sleep_milliseconds(opMode, 5000);

        /*DriveBuilderReset(false, false, "step");
        builder = builder.forward(24);
        trajectory = builder.build();   //x - 2.812, y + 7.984
        if (opMode.opModeIsActive())
            _drive.followTrajectorySync(trajectory);*/
        hwMap.frontRight.setPower(0.5);
        hwMap.frontLeft.setPower(0.5);
        hwMap.backRight.setPower(0.7);
        hwMap.backLeft.setPower(0.7);

        SafeSleep.sleep_milliseconds(opMode, 400);

        hwMap.frontRight.setPower(0);
        hwMap.frontLeft.setPower(0);
        hwMap.backRight.setPower(0);
        hwMap.backLeft.setPower(0);
    }

    public void BlueFoundationDrag() {
        transferReset(hwMap, opMode);
        initIntakeClaw(hwMap, opMode);
        startingPos = new Pose2d(new Vector2d(20.736, 63.936), Math.toRadians(270));

        SafeSleep.sleep_milliseconds(opMode,5000);

        _drive.getLocalizer().setPoseEstimate(startingPos);
        _drive.getLocalizer().update();
        builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
        builder = builder.lineTo(new Vector2d(20.736, 63.936)).lineTo(new Vector2d(20.736, 48.936))
                .strafeTo(new Vector2d(68.144, 48.936));
        trajectory = builder.build();
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);

        intake(hwMap, 0);
        //align.setPower(0.13, 0.25);
        //align.foundation(FieldPosition.BLUE_QUARY);
        transferReset(hwMap, opMode);

        _drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(68.144, 16.128), _drive.getExternalHeading()));
        _drive.getLocalizer().update();
        builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
        builder = builder.splineTo(new Pose2d(new Vector2d(37.064, 58.72), Math.toRadians(140)))
                .lineTo(new Vector2d(70.0, 58.72));
        trajectory = builder.build();
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);

        _drive.getLocalizer().setPoseEstimate(new Pose2d(new Vector2d(60.0, 54.72), Math.toRadians(0)));
        _drive.getLocalizer().update();
        builder = new TrajectoryBuilder(_drive.getPoseEstimate(), DriveConstants.BASE_CONSTRAINTS);
        builder = builder.lineTo(new Vector2d(54.0, 54.72)).strafeTo(new Vector2d(54.0, 60.44))
                .lineTo(new Vector2d(17.552, 60.44));
        trajectory = builder.build();
        if (opMode.opModeIsActive())
            _drive.followTrajectory(trajectory);
    }

        public void updateTFODData(List<Recognition> tfod) {
        this.tfod = tfod;
        updateTFOD();
    }

        public Pose2d getPoseEstimate() {
        return _drive.getLocalizer().getPoseEstimate();
    }


    private void updateTFOD() {

        //align.updateTFOD(tfod);
    }

    public void updateHeading() {
        //align.updateExternalHeading(Math.toDegrees(_drive.getExternalHeading()));
    }

        // TODO : servo updates are non-blocking, this is unnecessary
    public static void transferReset(AllHardwareMap hw, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
        mode) {
        LinearOpMode opmode = mode;
        Thread thread = new Thread() {
            public void run() {
                hw.transferHorn.setPosition(TeleopConstants.transferHornPosPush);
                SafeSleep.sleep_milliseconds(opmode, 200);
                hw.liftOdometry.setPosition(TeleopConstants.liftOdometryDown);
                SafeSleep.sleep_milliseconds(opmode, 200);
                //hwMap.innerTransfer.setPosition(TeleopConstants.innerTransferPosBlock);
            }
        };
        thread.start();
    }

    public static void initIntakeClaw(AllHardwareMap hw, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread t = new Thread() {
            public void run() {
                //hwMap.clawServo2.setPosition(0.9336);
                hw.clawServo1.setPosition(TeleopConstants.clawServo1PosClose);

                SafeSleep.sleep_milliseconds(opmode, 600);

                hw.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                SafeSleep.sleep_milliseconds(opmode, 600);

                //hwMap.clawInit.setPosition(TeleopConstants.clawInitPosReset);
                hw.clawInit.setPosition(TeleopConstants.clawInitPosReset);

                SafeSleep.sleep_milliseconds(opmode, 1000);

                hw.clawInit.setPosition(TeleopConstants.clawInitPosCapstone);

                SafeSleep.sleep_milliseconds(opmode, 600);

                hw.clawServo2.setPosition(TeleopConstants.clawServo2PosClose);

                SafeSleep.sleep_milliseconds(opmode, 200);
            }
        };
        t.start();
    }

    public static void prepGrab(AllHardwareMap hw, FieldPosition fieldPosition, boolean first, LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread t = new Thread() {
            public void run() {
                if (FieldPosition.RED_QUARY == fieldPosition) {
                    if (first) {
                        hw.redAutoClawJoint2.setPosition(0.85); //TODO Servo Test
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    } else {
                        SafeSleep.sleep_milliseconds(opmode, 1900);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        grabStone(hw, fieldPosition, opmode);
                    }
                } else {
                    if (first) {
                        hw.redAutoClawJoint2.setPosition(0.04);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    } else {
                        SafeSleep.sleep_milliseconds(opmode, 1900);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Prep_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Extended_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        grabStone(hw, fieldPosition, opmode);
                    }
                }
            }
        };
        t.start();

    }

    public static void grabStone(AllHardwareMap hw, FieldPosition fieldPosition, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
        mode) {
        LinearOpMode opmode = mode;
        if (FieldPosition.RED_QUARY == fieldPosition) {
            hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
            SafeSleep.sleep_milliseconds(opmode, 400);

            hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed);
            SafeSleep.sleep_milliseconds(opmode, 200);

            hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);
            SafeSleep.sleep_milliseconds(opmode, 200);

            hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);
            SafeSleep.sleep_milliseconds(opmode, 200);

        } else {
            hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing_blue);
            SafeSleep.sleep_milliseconds(opmode, 400);

            hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Closed_blue);
            SafeSleep.sleep_milliseconds(opmode, 200);

            hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp_blue);
            SafeSleep.sleep_milliseconds(opmode, 200);

            hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone_blue);
            SafeSleep.sleep_milliseconds(opmode, 200);

        }
    }

    public static void dropStone(AllHardwareMap hw, FieldPosition fieldPosition, boolean first, LinearOpMode mode) {
        LinearOpMode opmode = mode;
        Thread t = new Thread() {
            public void run() {
                SafeSleep.sleep_milliseconds(opmode, 2200);
                if (FieldPosition.RED_QUARY == fieldPosition) {
                    if (first) {
                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
                        SafeSleep.sleep_milliseconds(opmode, 400);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp);
                        SafeSleep.sleep_milliseconds(opmode, 400);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    } else {
                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing);
                        SafeSleep.sleep_milliseconds(opmode, 400);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.foundationLock.setPosition(TeleopConstants.foundationLockHalfUnlock);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.transferLock.setPosition(TeleopConstants.transferLockPosHalfUnlock);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    }
                } else {
                    if (first) {
                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop_blue);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2PickUp_blue);
                        SafeSleep.sleep_milliseconds(opmode, 400);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Stone_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    } else {
                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Drop_blue);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Grabbing_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Open_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint2.setPosition(TeleopConstants.autoClaw2Init_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint3.setPosition(TeleopConstants.autoClaw3Init_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.redAutoClawJoint1.setPosition(TeleopConstants.autoClaw1Retracted_blue);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.clawServo2.setPosition(TeleopConstants.clawServo2PosOpen);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.clawServo1.setPosition(TeleopConstants.clawServo1PosOpen);
                        SafeSleep.sleep_milliseconds(opmode, 600);

                        hw.foundationLock.setPosition(TeleopConstants.foundationLockHalfUnlock);
                        SafeSleep.sleep_milliseconds(opmode, 200);

                        hw.transferLock.setPosition(TeleopConstants.transferLockPosHalfUnlock);
                        SafeSleep.sleep_milliseconds(opmode, 200);
                    }
                }
            }
        };
        t.start();
    }

    public static void intake(AllHardwareMap hw, double power) {
        Thread thread = new Thread() {
            public void run() {
                hw.leftIntake.setPower(-power);
                hw.rightIntake.setPower(power);
            }
        };
        thread.start();
    }


    public static void init(AllHardwareMap hw, com.qualcomm.robotcore.eventloop.opmode.LinearOpMode mode) {
            LinearOpMode opmode =  mode;
            Thread thread = new Thread() {
                public void run() {
                    hw.parkingServo.setPosition(TeleopConstants.parkingServoPosLock);
                    SafeSleep.sleep_milliseconds(mode, 200);
                    hw.foundationLock.setPosition(TeleopConstants.foundationLockInit);
                    SafeSleep.sleep_milliseconds(mode, 200);
                    hw.transferLock.setPosition(TeleopConstants.transferLockPosOut);
                    SafeSleep.sleep_milliseconds(mode, 200);
                    hw.liftOdometry.setPosition(TeleopConstants.liftOdometryDown);
                    SafeSleep.sleep_milliseconds(mode, 200);
                    hw.innerTransfer.setPosition(TeleopConstants.innerTransferPosClosed);
                    SafeSleep.sleep_milliseconds(mode, 200);
                }
            };
            thread.start();
            SafeSleep.sleep_milliseconds(opmode, 50);
        }
    }
