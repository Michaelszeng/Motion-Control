package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.hardware.motors.GoBILDA5202Series;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import java.io.File;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    //"adb connect 192.168.43.1:5555"
    private static String TAG = "DriveConstants";
    public static boolean USE_VUFORIA_LOCALIZER = false;

    public static double odoTicksPerRevRight = 8114.5280;
    public static double odoTicksPerRevLeft = 8133.0806;
    public static double odoTicksPerRevHorizontal = 8104.8806;
    public static double odoEncoderTicksPerRev = 1565.0;
    public static double odoWheelRadius = 1.1811;
    public static double txP = 5.0; //translational x/y co-efficients
    public static double txI = 0.5;
    public static double txD = 0.0;
    public static double tyP = 5.0;
    public static double tyI = 10.0;
    public static double tyD = 0.00001;
    public static double hP = 10;    // heading co-efficients;
    public static double hI = 0.5;
    public static double hD = 0.00001;

    public static double ODOMETRY_TRACK_WIDTH = 14.632;
    public static double ODOMETRY_HORIZONTAL_TRACK_WIDTH = -4.4;
    public static double HARDCODED_TICKS_PER_REV = 383.6; //MOTOR_CONFIG.getTicksPerRev();
    public static double MAX_RPM_FROM_SPEC = 435.0;
    public static double HARDCODED_RPM_RATIO = 0.683; //0.72215; // 0.666;///0.6514;//*MAX_RPM_FROM_SPEC; //283.4; //MOTOR_CONFIG.getMaxRPM();
    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 1;
    public static final double MAX_RPM = 1;
    private static final MotorConfigurationType MOTOR_CONFIG =
            MotorConfigurationType.getMotorType(GoBILDA5202Series.class);// NeveRest20Gearmotor
    // Matrix12vMotor  GoBILDA5202Series MatrixLegacyMotor (757.12)


    /*
     * Set the first flag appropriately. If using the built-in motor velocity PID, update
     * MOTOR_VELO_PID with the tuned coefficients from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = true;
    public static double kP = 1.72;
    public static double kI = 0.172;
    public static double kD = 0.0;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(kP, kI, kD);   //35, 0.5, 2.5


    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 2;
    public static double GEAR_RATIO = 1.0;//(99.5 / 13.7) * (16 / 16); // output (wheel) speed / input (motor) speed
    public static double TRACK_WIDTH = 14.2;   //17
    public static double WHEEL_BASE = 14.2;   //???


    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0111;   //0.0115
    //public static double kV = 1.0 / rpmToVelocity(MAX_RPM);

    public static double kA = 0;
    public static double kStatic = 0;

    public static double maxVel = 75.0; //90.0
    public static double maxAccel = 35.0;   //35.0
    public static double maxAngVel = 135.0;
    public static double maxAngAccel = 90.0;
    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    public static DriveConstraints BASE_CONSTRAINTS = new DriveConstraints(
            maxVel, maxAccel, 0.0,
            Math.toRadians(maxAngVel), Math.toRadians(maxAngAccel), 0.0
    );




    public static double encoderTicksToInches(double ticks) {
        //double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
        double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / HARDCODED_TICKS_PER_REV; //MOTOR_CONFIG.getTicksPerRev();
        //RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double odoTicksToInchesRight(double ticks) {
        //double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
        double s = odoWheelRadius * 2 * Math.PI * ticks / odoTicksPerRevRight; //MOTOR_CONFIG.getTicksPerRev();
        //RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double odoTicksToInchesLeft(double ticks) {
        //double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
        double s = odoWheelRadius * 2 * Math.PI * ticks / odoTicksPerRevLeft; //MOTOR_CONFIG.getTicksPerRev();
        //RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }

    public static double odoTicksToInchesHorizontal(double ticks) {
        //double s = WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / MOTOR_CONFIG.getTicksPerRev();
        double s = odoWheelRadius * 2 * Math.PI * ticks / odoTicksPerRevHorizontal; //MOTOR_CONFIG.getTicksPerRev();
        //RobotLog.dd(TAG, "encoderTicksToInches: " + "ticks: " + Double.toString(ticks) + " inches: " + Double.toString(s));
        return s;
    }



    public static double rpmToVelocity(double rpm) {
        double s = rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
        RobotLogger.dd(TAG, "rpmToVelocity: " + "rpm " + Double.toString(rpm) + " v " + Double.toString(s));
        return s;
    }

    public static double odoRpmToVelocity(double rpm) {
        double s = rpm * 2 * Math.PI * odoWheelRadius / 60.0;
        RobotLogger.dd(TAG, "rpmToVelocity: " + "rpm " + Double.toString(rpm) + " v " + Double.toString(s));
        return s;
    }



    public static double getMaxRpm() {
        RobotLogger.dd(TAG, "MOTOR_CONFIG.getAchieveableMaxRPMFraction(): " + Double.toString(MOTOR_CONFIG.getAchieveableMaxRPMFraction()));
        RobotLogger.dd(TAG, "MOTOR_CONFIG.getMaxRPM(): " + Double.toString(MOTOR_CONFIG.getMaxRPM()));
        double t = MOTOR_CONFIG.getMaxRPM() *
                (RUN_USING_ENCODER ? MOTOR_CONFIG.getAchieveableMaxRPMFraction() : 1.0);
        t = MAX_RPM_FROM_SPEC * (RUN_USING_ENCODER ? HARDCODED_RPM_RATIO : 1.0);
        RobotLogger.dd(TAG, "getMaxRpm: hardcoded to: "+Double.toString((t))+" from: "+Double.toString(MAX_RPM_FROM_SPEC));
        return t;
    }

    public static double getTicksPerSec() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = getMaxRpm() * HARDCODED_TICKS_PER_REV / 60.0;
        RobotLogger.dd(TAG,  "getTicksPerSec "+Double.toString(t));
        return t;
    }

    public static double getOdoTicksPerSecLeft() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = getMaxRpm() * odoTicksPerRevLeft / 60.0;
        RobotLogger.dd(TAG,  "getOdoTicksPerSecLeft "+Double.toString(t));
        return t;
    }

    public static double getOdoTicksPerSecRight() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = getMaxRpm() * odoTicksPerRevRight / 60.0;
        RobotLogger.dd(TAG,  "getOdoTicksPerSecLeft "+Double.toString(t));
        return t;
    }

    public static double getOdoTicksPerSecHorizonal() {
        // note: MotorConfigurationType#getAchieveableMaxTicksPerSecond() isn't quite what we want
        //double t = MOTOR_CONFIG.getMaxRPM() * MOTOR_CONFIG.getTicksPerRev() / 60.0;
        //double t = MOTOR_CONFIG.getMaxRPM() * HARDCODED_TICKS_PER_REV / 60.0;
        double t = getMaxRpm() * odoTicksPerRevHorizontal / 60.0;
        RobotLogger.dd(TAG,  "getOdoTicksPerSecHorizontal "+Double.toString(t));
        return t;
    }



    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        RobotLogger.dd(TAG, "getTicksPerSec "+Double.toString(getTicksPerSec()));
        return 32767 / getTicksPerSec();
    }
    public static Pose2d[] parsePathXY(String filename) {
        String full_path = AppUtil.CONFIG_FILES_DIR + "/" + filename;
        RobotLogger.dd(TAG, "path definition file: " + full_path);
        Pose2d[] coordinates = null;

        try {
            File inputFile = new File(AppUtil.CONFIG_FILES_DIR+"/"+filename);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document doc = dBuilder.parse(inputFile);
            doc.getDocumentElement().normalize();
            RobotLogger.dd(TAG, "Root element :" + doc.getDocumentElement().getNodeName());
            NodeList nList = doc.getElementsByTagName("movestep");
            RobotLogger.dd(TAG,"----------------------------");
            int step_num = nList.getLength();
            RobotLogger.dd(TAG, "elements :" + Integer.toString(step_num));

            coordinates = new Pose2d[step_num];

            for (int temp = 0; temp < nList.getLength(); temp++) {
                Node nNode = nList.item(temp);
                //RobotLogger.dd(TAG, "Current Element :" + nNode.getNodeName());

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {
                    double x, y, h;
                    Element eElement = (Element) nNode;
                    RobotLogger.dd(TAG, "step info : "
                            + eElement.getAttribute("drive_type"));
                    x = new Double(eElement
                            .getElementsByTagName("x")
                            .item(0)
                            .getTextContent());
                    y = new Double(eElement
                            .getElementsByTagName("y")
                            .item(0)
                            .getTextContent());
                    h = new Double(eElement
                            .getElementsByTagName("h")
                            .item(0)
                            .getTextContent());
                    RobotLogger.dd(TAG,"step %d: x: %f, y: %f, h: %f", temp, x, y , h);
                    coordinates[temp] = new Pose2d(x, y, Math.toRadians(h));
                }
            }
        } catch (Exception e) {
            RobotLogger.dd(TAG, "cannot find path XY file: " + full_path);
            e.printStackTrace();
        }
        return coordinates;
    }

    public static boolean ENABLE_LOGGING = true;
    public static boolean VirtualizeDrive = true;
}
