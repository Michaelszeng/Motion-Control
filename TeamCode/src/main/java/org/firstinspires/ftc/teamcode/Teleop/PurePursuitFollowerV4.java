package org.firstinspires.ftc.teamcode.Teleop;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.control.PurePursuitMotionProfileGenerator;
import org.firstinspires.ftc.teamcode.drive.virtual.FieldDashboard;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.PurePursuitMathFunctions;
import org.firstinspires.ftc.teamcode.util.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.List;

import com.qualcomm.robotcore.util.ReadWriteFile;
import java.io.FileWriter;
import java.io.BufferedWriter;
import java.io.File;  // Import the File class
import java.io.IOException;  // Import the IOException class to handle errors
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;

import static org.firstinspires.ftc.teamcode.util.PurePursuitMathFunctions.getNextTargetV5;
import static org.firstinspires.ftc.teamcode.util.PurePursuitMathFunctions.reachedDestination;

/*
 * Follows a path using pure pursuit algorithm
 *
 * THIS VERSION IS UNDERGOING TESTING WITH TIME BASED MOTION PROFILING
 */
@TeleOp(name = "PurePursuitFollowerV4")
public class PurePursuitFollowerV4 extends LinearOpMode {
    String TAG = "PurePursuitFollowerV4";

    AllHardwareMap hwMap;
    //frontRight: horizontal odometer
    //backRight: vertical right odometer
    //frontLeft: front left motor
    //backLeft: vertical Left odometer
    DcMotor frontRight, backRight, frontLeft, backLeft;

    BNO055IMU imu;

    double prevDuration = -999999;      //Set this to a very large number so the starting v and a turn out to be 0
    double duration = 0.0;
    Date datePrev = new Date();
    Date dateNew = new Date();
    Date startDate = new Date();
    double dateDiff;

    FtcDashboard dashboard;

    double radius = 12;

    PurePursuitPath PPPath;
    String pathString = "0.0,0.0; 0.0,0.2057142857142793; 0.0,0.4114285714285728; 0.0,0.6171428571428521; 0.0,0.8228571428571456; 0.0,1.028571428571425; 0.0,1.2342857142857184; 0.0,1.4399999999999977; 0.0,1.6457142857142912; 0.0,1.8514285714285705; 0.0,2.057142857142864; 0.0,2.2628571428571433; 0.0,2.4685714285714226; 0.0,2.674285714285716; 0.0,2.8799999999999955; 0.0,3.085714285714289; 0.0,3.2914285714285683; 0.0,3.4971428571428618; 0.0,3.702857142857141; 0.0,3.9085714285714346; 0.0,4.114285714285714; 0.0,4.319999999999993; 0.0,4.525714285714287; 0.0,4.731428571428566; 0.0,4.9371428571428595; 0.0,5.142857142857139; 0.0,5.348571428571432; 0.0,5.554285714285712; 0.0,5.760000000000005; 0.0,5.965714285714284; 0.0,6.171428571428578; 0.0,6.377142857142857; 0.0,6.5828571428571365; 0.0,6.78857142857143; 0.0,6.994285714285709; 0.0,7.200000000000003; 0.0,7.405714285714282; 0.0,7.611428571428576; 0.0,7.817142857142855; 0.0,8.022857142857141; 0.0,8.228571428571428; 0.0,8.434285714285714; 0.0,8.64; 0.0,8.845714285714287; 0.0,9.051428571428573; 0.0,9.25714285714286; 0.0,9.462857142857146; 0.0,9.668571428571425; 0.0,9.874285714285712; 0.0,10.079999999999998; 0.0,10.285714285714285; 0.0,10.491428571428571; 0.0,10.697142857142858; 0.0,10.902857142857144; 0.0,11.10857142857143; 0.0,11.314285714285717; 0.0,11.520000000000003; 0.0,11.725714285714282; 0.0,11.931428571428569; 0.0,12.137142857142855; 0.0,12.342857142857142; 0.0,12.548571428571428; 0.0,12.754285714285714; 0.0,12.96; 0.0,13.165714285714287; 0.0,13.371428571428574; 0.0,13.57714285714286; 0.0,13.78285714285714; 0.0,13.988571428571426; 0.0,14.194285714285712; 0.0,14.399999999999999; 0.0,14.605714285714285; 0.0,14.811428571428571; 0.0,15.017142857142858; 0.0,15.222857142857144; 0.0,15.42857142857143; 0.0,15.634285714285717; 0.0,15.840000000000003; 0.0,16.045714285714283; 0.0,16.25142857142857; 0.0,16.457142857142856; 0.0,16.662857142857142; 0.0,16.86857142857143; 0.0,17.074285714285715; 0.0,17.28; 0.0,17.485714285714288; 0.0,17.691428571428574; 0.0,17.89714285714286; 0.0,18.10285714285714; 0.0,18.308571428571426; 0.0,18.514285714285712; 0.0,18.72; 0.0,18.925714285714285; 0.0,19.13142857142857; 0.0,19.337142857142858; 0.0,19.542857142857144; 0.0,19.74857142857143; 0.0,19.954285714285717; 0.0,20.159999999999997; 0.0,20.365714285714283; 0.0,20.57142857142857; 0.0,20.777142857142856; 0.0,20.982857142857142; 0.0,21.18857142857143; 0.0,21.394285714285715; 0.0,21.6; 0.0,21.805714285714288; 0.0,22.011428571428574; 0.0,22.21714285714286; 0.0,22.42285714285714; 0.0,22.628571428571426; 0.0,22.834285714285713; 0.0,23.04; 0.0,23.245714285714286; 0.0,23.451428571428572; 0.0,23.65714285714286; 0.0,23.862857142857145; 0.0,24.06857142857143; 0.0,24.274285714285718; 0.0,24.479999999999997; 0.0,24.685714285714283; 0.0,24.89142857142857; 0.0,25.097142857142856; 0.0,25.302857142857142; 0.0,25.50857142857143; 0.0,25.714285714285715; 0.0,25.92; 0.0,26.125714285714288; 0.0,26.331428571428575; 0.0,26.537142857142854; 0.0,26.74285714285714; 0.0,26.948571428571427; 0.0,27.154285714285713; 0.0,27.36; 0.0,27.565714285714286; 0.0,27.771428571428572; 0.0,27.97714285714286; 0.0,28.182857142857145; 0.0,28.38857142857143; 0.0,28.59428571428571; 0.0,28.799999999999997; 0.0,29.005714285714284; 0.0,29.21142857142857; 0.0,29.417142857142856; 0.0,29.622857142857143; 0.0,29.82857142857143; 0.0,30.034285714285716; 0.0,30.240000000000002; 0.0,30.44571428571429; 0.0,30.651428571428575; 0.0,30.857142857142854; 0.0,31.06285714285714; 0.0,31.268571428571427; 0.0,31.474285714285713; 0.0,31.68; 0.0,31.885714285714286; 0.0,32.09142857142857; 0.0,32.29714285714286; 0.0,32.502857142857145; 0.0,32.70857142857143; 0.0,32.91428571428571; 0.0,33.12; 0.0,33.325714285714284; 0.0,33.53142857142857; 0.0,33.73714285714286; 0.0,33.94285714285714; 0.0,34.14857142857143; 0.0,34.354285714285716; 0.0,34.56; 0.0,34.76571428571429; 0.0,34.97142857142857; 0.0,35.177142857142854; 0.0,35.38285714285714; 0.0,35.58857142857143; 0.0,35.794285714285714; 0.0,36.0; 0.0,36.205714285714286; 0.0,36.41142857142857; 0.0,36.61714285714286; 0.0,36.822857142857146; 0.0,37.02857142857143; 0.0,37.23428571428571; 0.0,37.44; 0.0,37.645714285714284; 0.0,37.85142857142857; 0.0,38.05714285714286; 0.0,38.26285714285714; 0.0,38.46857142857143; 0.0,38.674285714285716; 0.0,38.88; 0.0,39.08571428571429; 0.0,39.29142857142857; 0.0,39.497142857142855; 0.0,39.70285714285714; 0.0,39.90857142857143; 0.0,40.114285714285714; 0.0,40.32; 0.0,40.52571428571429; 0.0,40.73142857142857; 0.0,40.93714285714286; 0.0,41.14285714285714; 0.0,41.34857142857143; 0.0,41.55428571428571; 0.0,41.760000000000005; 0.0,41.965714285714284; 0.0,42.17142857142857; 0.0,42.37714285714286; 0.0,42.582857142857144; 0.0,42.78857142857143; 0.0,42.99428571428571; 0.0,43.2; 0.0,43.40571428571428; 0.0,43.611428571428576; 0.0,43.817142857142855; 0.0,44.02285714285714; 0.0,44.22857142857143; 0.0,44.434285714285714; 0.0,44.64; 0.0,44.84571428571429; 0.0,45.05142857142857; 0.0,45.25714285714285; 0.0,45.462857142857146; 0.0,45.668571428571425; 0.0,45.87428571428572; 0.0,46.08; 0.0,46.285714285714285; 0.0,46.49142857142857; 0.0,46.69714285714286; 0.0,46.902857142857144; 0.0,47.10857142857143; 0.0,47.31428571428572; 0.0,47.519999999999996; 0.0,47.72571428571429; 0.0,47.93142857142857; 0.0,48.13714285714286; 0.0,48.34285714285714; 0.0,48.54857142857143; 0.0,48.754285714285714; 0.0,48.96; 0.0,49.16571428571429; 0.0,49.37142857142857; 0.0,49.57714285714286; 0.0,49.78285714285714; 0.0,49.98857142857143; 0.0,50.19428571428571; 0.0,50.4; 0.0,50.605714285714285; 0.0,50.81142857142857; 0.0,51.01714285714286; 0.0,51.222857142857144; 0.0,51.42857142857143; 0.0,51.63428571428571; 0.0,51.84; 0.0,52.04571428571428; 0.0,52.251428571428576; 0.0,52.457142857142856; 0.0,52.66285714285714; 0.0,52.86857142857143; 0.0,53.074285714285715; 0.0,53.28; 0.0,53.48571428571428; 0.0,53.691428571428574; 0.0,53.89714285714285; 0.0,54.10285714285715; 0.0,54.308571428571426; 0.0,54.51428571428572; 0.0,54.72; 0.0,54.925714285714285; 0.0,55.13142857142857; 0.0,55.33714285714286; 0.0,55.542857142857144; 0.0,55.748571428571424; 0.0,55.95428571428572; 0.0,56.16; 0.0,56.36571428571429; 0.0,56.57142857142857; 0.0,56.777142857142856; 0.0,56.98285714285714; 0.0,57.18857142857143; 0.0,57.18857142857143; 0.2057142857142793,57.18857142857143; 0.4114285714285728,57.18857142857143; 0.6171428571428521,57.18857142857143; 0.8228571428571456,57.18857142857143; 1.028571428571425,57.18857142857143; 1.2342857142857184,57.18857142857143; 1.4399999999999977,57.18857142857143; 1.6457142857142912,57.18857142857143; 1.8514285714285705,57.18857142857143; 2.057142857142864,57.18857142857143; 2.2628571428571433,57.18857142857143; 2.4685714285714226,57.18857142857143; 2.674285714285716,57.18857142857143; 2.8799999999999955,57.18857142857143; 3.085714285714289,57.18857142857143; 3.2914285714285683,57.18857142857143; 3.4971428571428618,57.18857142857143; 3.702857142857141,57.18857142857143; 3.9085714285714346,57.18857142857143; 4.114285714285714,57.18857142857143; 4.319999999999993,57.18857142857143; 4.525714285714287,57.18857142857143; 4.731428571428566,57.18857142857143; 4.9371428571428595,57.18857142857143; 5.142857142857139,57.18857142857143; 5.348571428571432,57.18857142857143; 5.554285714285712,57.18857142857143; 5.760000000000005,57.18857142857143; 5.965714285714284,57.18857142857143; 6.171428571428578,57.18857142857143; 6.377142857142857,57.18857142857143; 6.5828571428571365,57.18857142857143; 6.78857142857143,57.18857142857143; 6.994285714285709,57.18857142857143; 7.200000000000003,57.18857142857143; 7.405714285714282,57.18857142857143; 7.611428571428576,57.18857142857143; 7.817142857142855,57.18857142857143; 8.022857142857148,57.18857142857143; 8.228571428571428,57.18857142857143; 8.434285714285721,57.18857142857143; 8.64,57.18857142857143; 8.84571428571428,57.18857142857143; 9.051428571428573,57.18857142857143; 9.257142857142853,57.18857142857143; 9.462857142857146,57.18857142857143; 9.668571428571425,57.18857142857143; 9.874285714285719,57.18857142857143; 10.079999999999998,57.18857142857143; 10.285714285714292,57.18857142857143; 10.491428571428571,57.18857142857143; 10.69714285714285,57.18857142857143; 10.902857142857144,57.18857142857143; 11.108571428571423,57.18857142857143; 11.314285714285717,57.18857142857143; 11.519999999999996,57.18857142857143; 11.72571428571429,57.18857142857143; 11.931428571428569,57.18857142857143; 12.137142857142862,57.18857142857143; 12.342857142857142,57.18857142857143; 12.548571428571435,57.18857142857143; 12.754285714285714,57.18857142857143; 12.959999999999994,57.18857142857143; 13.165714285714287,57.18857142857143; 13.371428571428567,57.18857142857143; 13.57714285714286,57.18857142857143; 13.78285714285714,57.18857142857143; 13.988571428571433,57.18857142857143; 14.194285714285712,57.18857142857143; 14.400000000000006,57.18857142857143; 14.605714285714285,57.18857142857143; 14.811428571428578,57.18857142857143; 15.017142857142858,57.18857142857143; 15.222857142857137,57.18857142857143; 15.42857142857143,57.18857142857143; 15.63428571428571,57.18857142857143; 15.840000000000003,57.18857142857143; 16.045714285714283,57.18857142857143; 16.251428571428576,57.18857142857143; 16.457142857142856,57.18857142857143; 16.66285714285715,57.18857142857143; 16.86857142857143,57.18857142857143; 17.074285714285708,57.18857142857143; 17.28,57.18857142857143; 17.48571428571428,57.18857142857143; 17.691428571428574,57.18857142857143; 17.897142857142853,57.18857142857143; 18.102857142857147,57.18857142857143; 18.308571428571426,57.18857142857143; 18.51428571428572,57.18857142857143; 18.72,57.18857142857143; 18.925714285714292,57.18857142857143; 19.13142857142857,57.18857142857143; 19.33714285714285,57.18857142857143; 19.542857142857144,57.18857142857143; 19.748571428571424,57.18857142857143; 19.954285714285717,57.18857142857143; 20.159999999999997,57.18857142857143; 20.36571428571429,57.18857142857143; 20.57142857142857,57.18857142857143; 20.777142857142863,57.18857142857143; 20.982857142857142,57.18857142857143; 21.18857142857142,57.18857142857143; 21.394285714285715,57.18857142857143; 21.599999999999994,57.18857142857143; 21.805714285714288,57.18857142857143; 22.011428571428567,57.18857142857143; 22.21714285714286,57.18857142857143; 22.42285714285714,57.18857142857143; 22.628571428571433,57.18857142857143; 22.834285714285713,57.18857142857143; 23.040000000000006,57.18857142857143; 23.245714285714286,57.18857142857143; 23.451428571428565,57.18857142857143; 23.65714285714286,57.18857142857143; 23.862857142857138,57.18857142857143; 24.06857142857143,57.18857142857143; 24.27428571428571,57.18857142857143; 24.480000000000004,57.18857142857143; 24.685714285714283,57.18857142857143; 24.891428571428577,57.18857142857143; 25.097142857142856,57.18857142857143; 25.30285714285715,57.18857142857143; 25.50857142857143,57.18857142857143; 25.714285714285708,57.18857142857143; 25.92,57.18857142857143; 26.12571428571428,57.18857142857143; 26.331428571428575,57.18857142857143; 26.537142857142854,57.18857142857143; 26.742857142857147,57.18857142857143; 26.948571428571427,57.18857142857143; 27.15428571428572,57.18857142857143; 27.36,57.18857142857143; 27.56571428571428,57.18857142857143; 27.771428571428572,57.18857142857143; 27.97714285714285,57.18857142857143; 28.182857142857145,57.18857142857143; 28.388571428571424,57.18857142857143; 28.594285714285718,57.18857142857143; 28.799999999999997,57.18857142857143; 29.00571428571429,57.18857142857143; 29.21142857142857,57.18857142857143; 29.417142857142863,57.18857142857143; 29.622857142857143,57.18857142857143; 29.828571428571422,57.18857142857143; 30.034285714285716,57.18857142857143; 30.239999999999995,57.18857142857143; 30.44571428571429,57.18857142857143; 30.651428571428568,57.18857142857143; 30.85714285714286,57.18857142857143; 31.06285714285714,57.18857142857143; 31.268571428571434,57.18857142857143; 31.474285714285713,57.18857142857143; 31.680000000000007,57.18857142857143; 31.885714285714286,57.18857142857143; 32.091428571428565,57.18857142857143; 32.29714285714286,57.18857142857143; 32.50285714285714,57.18857142857143; 32.70857142857143,57.18857142857143; 32.91428571428571,57.18857142857143; 33.120000000000005,57.18857142857143; 33.325714285714284,57.18857142857143; 33.53142857142858,57.18857142857143; 33.73714285714286,57.18857142857143; 33.942857142857136,57.18857142857143; 34.14857142857143,57.18857142857143; 34.35428571428571,57.18857142857143; 34.56,57.18857142857143; 34.76571428571428,57.18857142857143; 34.971428571428575,57.18857142857143; 35.177142857142854,57.18857142857143; 35.38285714285715,57.18857142857143; 35.58857142857143,57.18857142857143; 35.79428571428572,57.18857142857143; 36.0,57.18857142857143; 36.20571428571428,57.18857142857143; 36.41142857142857,57.18857142857143; 36.61714285714285,57.18857142857143; 36.822857142857146,57.18857142857143; 37.028571428571425,57.18857142857143; 37.23428571428572,57.18857142857143; 37.44,57.18857142857143; 37.64571428571429,57.18857142857143; 37.85142857142857,57.18857142857143; 38.057142857142864,57.18857142857143; 38.26285714285714,57.18857142857143; 38.46857142857142,57.18857142857143; 38.674285714285716,57.18857142857143; 38.879999999999995,57.18857142857143; 39.08571428571429,57.18857142857143; 39.29142857142857,57.18857142857143; 39.49714285714286,57.18857142857143; 39.70285714285714,57.18857142857143; 39.908571428571435,57.18857142857143; 40.114285714285714,57.18857142857143; 40.31999999999999,57.18857142857143; 40.52571428571429,57.18857142857143; 40.731428571428566,57.18857142857143; 40.93714285714286,57.18857142857143; 41.14285714285714,57.18857142857143; 41.34857142857143,57.18857142857143; 41.55428571428571,57.18857142857143; 41.760000000000005,57.18857142857143; 41.965714285714284,57.18857142857143; 42.17142857142858,57.18857142857143; 42.37714285714286,57.18857142857143; 42.58285714285714,57.18857142857143; 42.78857142857143,57.18857142857143; 42.99428571428571,57.18857142857143; 43.2,57.18857142857143; 43.40571428571428,57.18857142857143; 43.611428571428576,57.18857142857143; 43.817142857142855,57.18857142857143; 44.02285714285715,57.18857142857143; 44.22857142857143,57.18857142857143; 44.43428571428572,57.18857142857143; 44.64,57.18857142857143; 44.84571428571428,57.18857142857143; 45.05142857142857,57.18857142857143; 45.25714285714285,57.18857142857143; 45.462857142857146,57.18857142857143; 45.668571428571425,57.18857142857143; 45.87428571428572,57.18857142857143; 46.08,57.18857142857143; 46.28571428571429,57.18857142857143; 46.49142857142857,57.18857142857143; 46.69714285714285,57.18857142857143; 46.902857142857144,57.18857142857143; 47.10857142857142,57.18857142857143; 47.31428571428572,57.18857142857143; 47.519999999999996,57.18857142857143; 47.72571428571429,57.18857142857143; 47.93142857142857,57.18857142857143; 48.13714285714286,57.18857142857143; 48.34285714285714,57.18857142857143; 48.548571428571435,57.18857142857143; 48.754285714285714,57.18857142857143; 48.959999999999994,57.18857142857143; 49.16571428571429,57.18857142857143; 49.37142857142857,57.18857142857143; 49.57714285714286,57.18857142857143; 49.78285714285714,57.18857142857143; 49.98857142857143,57.18857142857143; 50.19428571428571,57.18857142857143; 50.400000000000006,57.18857142857143; 50.605714285714285,57.18857142857143; 50.81142857142858,57.18857142857143; 51.01714285714286,57.18857142857143; 51.22285714285714,57.18857142857143; 51.42857142857143,57.18857142857143; 51.63428571428571,57.18857142857143; 51.84,57.18857142857143; 52.04571428571428,57.18857142857143; 52.251428571428576,57.18857142857143; 52.457142857142856,57.18857142857143; 52.66285714285715,57.18857142857143; 52.86857142857143,57.18857142857143; 53.07428571428571,57.18857142857143; 53.28,57.18857142857143; 53.48571428571428,57.18857142857143; 53.691428571428574,57.18857142857143; 53.89714285714285,57.18857142857143; 54.10285714285715,57.18857142857143; 54.308571428571426,57.18857142857143; 54.51428571428572,57.18857142857143; 54.72,57.18857142857143; 54.92571428571429,57.18857142857143; 54.92571428571429,57.18857142857143;";
    ArrayList<Pose2d> path = parsePath(pathString);
    PurePursuitPathPoint pathEnd;

    int prevTargetIndex = 0;
    PurePursuitMathFunctions.ReachedDestination reachedDestination = PurePursuitMathFunctions.ReachedDestination.FALSE;

    ArrayList<Double> tHist = new ArrayList<>(); ArrayList<Double> xHist = new ArrayList<>(); ArrayList<Double> yHist = new ArrayList<>(); ArrayList<Double> hHist = new ArrayList<>(); ArrayList<Double> vHist = new ArrayList<>(); ArrayList<Double> aHist = new ArrayList<>(); ArrayList<Double> angVHist = new ArrayList<>(); ArrayList<Double> angAHist = new ArrayList<>();
    double prevX = 0.0;
    double prevY = 0.0;
    double prevH = 0.0;
    double prevV = 0.0;
    double prevAngV = 0.0;
    File logFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    String MPString;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotLogger.dd(TAG, "_______________________________________________________");
        PPPath = new PurePursuitPath(path, "standard_test_v_path.xml");
        RobotLogger.dd(TAG, "PPPath.size(): " + PPPath.path1.size());
//        for (PurePursuitPathPoint p : PPPath.path1) {
//            p.toString();
//        }
        pathEnd = PPPath.path1.get(PPPath.path1.size()-1);

        PurePursuitMotionProfileGenerator mpg = new PurePursuitMotionProfileGenerator(PPPath);
        mpg.generateProfiles();
        mpg.exportProfiles();

//        RobotLogger.dd(TAG, PPPath.toString());
        MPString = PPPath.toString();

        hwMap = new AllHardwareMap(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

//        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
//        frontRight = hwMap.frontRight;
//        backRight = hwMap.backRight;
//        frontLeft = hwMap.frontLeft;
//        backLeft = hwMap.backLeft;


        //BULK READ PARAMETERS
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        //END BULK READ PARAMETERS


        initHardwareMap(frontRight, backRight, frontLeft, backLeft);
        //Initialize IMU parameters
        imu = hwMap.gyro;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        Robot robot = new Robot(hardwareMap, false,0.0, 0.0, 0.0);
        ArrayList<Double> motorPowers;
        Pose2d currentTarget;
        int targetIndex;

        FieldDashboard fieldDashboard = new FieldDashboard(robot);

        Pose2d currentPose;

        tHist.add(0.0);
        xHist.add(0.0);
        yHist.add(0.0);
        hHist.add(0.0);
        vHist.add(0.0);
        aHist.add(0.0);
        angVHist.add(0.0);
        angAHist.add(0.0);


        waitForStart();
        startDate.setTime(new Date().getTime());
        while (opModeIsActive()) {   //put teleop code in here
            dateNew.setTime(new Date().getTime());
            dateDiff = dateNew.getTime() - datePrev.getTime();
            datePrev.setTime(dateNew.getTime());
            duration = dateNew.getTime() - startDate.getTime();
            RobotLogger.dd(TAG, "Current Duration: " + duration);

            currentPose = robot.getCurrentPose();
            RobotLogger.dd(TAG, "localizer: (" + currentPose.getX() + ", " + currentPose.getY() + ", " + Math.toDegrees(currentPose.getHeading()) + ")");

            targetIndex = getNextTargetV5(currentPose, radius, PPPath, prevTargetIndex);
            currentTarget = new Pose2d(PPPath.path1.get(targetIndex).x, PPPath.path1.get(targetIndex).y, PPPath.path1.get(targetIndex).h);
            prevTargetIndex = targetIndex;
            RobotLogger.dd(TAG, "currentTarget: (" + currentTarget.getX() + ", " + currentTarget.getY() + ", " + Math.toDegrees(currentTarget.getHeading()) + ")");

            double imuReading = -imu.getAngularOrientation().firstAngle;
//            imuReading = getHeading(targetIndex, PPPath);
//            RobotLogger.dd(TAG, "Corrected imuReading: " + imuReading);
//            motorPowers = robot.update(backRight.getCurrentPosition(), backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), imuReading, currentTarget, (int) dateDiff);
            motorPowers = robot.update(backRight.getCurrentPosition(), backLeft.getCurrentPosition(), frontRight.getCurrentPosition(), imuReading, currentTarget, targetIndex, PPPath, (int) dateDiff, duration/1000);
            detectEnd(currentPose);

            //FL, BL, BR, FR
            robot.setMotorPowers(motorPowers.get(0), motorPowers.get(1), motorPowers.get(2), motorPowers.get(3));

            tHist.add(duration);
            xHist.add(currentPose.getX());
            yHist.add(currentPose.getY());
            hHist.add(currentPose.getHeading());
            vHist.add(Math.hypot(currentPose.getX() - prevX, currentPose.getY() - prevY) / ((duration - prevDuration)/1000));
            angVHist.add((currentPose.getHeading() - prevH) / ((duration - prevDuration)/1000));
            aHist.add((vHist.get(vHist.size()-1) - prevV) / ((duration - prevDuration)/1000));
            angAHist.add((angVHist.get(angVHist.size()-1) - prevAngV) / ((duration - prevDuration)/1000));
            prevDuration = duration;
            prevX = currentPose.getX();
            prevY = currentPose.getY();
            prevH = currentPose.getHeading();
            prevV = vHist.get(vHist.size()-1);
            prevAngV = vHist.get(vHist.size()-1);

            Log.d(TAG, "Loopcycle: " + dateDiff);
            telemetry.addData("Localizer: ", "(" + currentPose.getX() + ", " + currentPose.getY() + ", " + Math.toDegrees(currentPose.getHeading()) + ")");
            telemetry.update();
            fieldDashboard.updateDashboard();
//            SafeSleep.sleep_milliseconds(this, 20);
            idle();
        }
    }

    public double getHeading(int targetIndex, PurePursuitPath PPPath) {
        double imuReading = -imu.getAngularOrientation().firstAngle;
        double imuReading2 = -imu.getAngularOrientation().secondAngle;
        double imuReading3 = -imu.getAngularOrientation().thirdAngle;
        int differenceInRobotPoseIndexAndTargetIndex = 40;
        double approxPathPointHeading = PPPath.path1.get(targetIndex - differenceInRobotPoseIndexAndTargetIndex).h;

        double minDifference = 999.9;
        double minDifferenceReading = imuReading;
        if (Math.abs(imuReading - approxPathPointHeading)  < minDifference) {
            minDifference = Math.abs(imuReading - approxPathPointHeading);
            minDifferenceReading = imuReading;
        }
        if (Math.abs(imuReading2 - approxPathPointHeading)  < minDifference) {
            minDifference = Math.abs(imuReading2 - approxPathPointHeading);
            minDifferenceReading = imuReading2;
        }
        if (Math.abs(imuReading3 - approxPathPointHeading)  < minDifference) {
//            minDifference = Math.abs(imuReading3 - approxPathPointHeading);
            minDifferenceReading = imuReading3;
        }
        return minDifferenceReading;
    }

    public ArrayList<Pose2d> parsePath(String pathString) {
        List<String> split = Arrays.asList(pathString.split("; "));
        ArrayList<Pose2d> path = new ArrayList<>();
        for (int i=0; i<split.size(); i++) {
            String[] point = split.get(i).split(",");

            double x = Double.parseDouble(point[0].replaceAll(";", ""));
            double y = Double.parseDouble(point[1].replaceAll(";", ""));
            path.add(new Pose2d(x, y, 0.0));
        }
        return path;
    }

    private void initHardwareMap(DcMotor right_front, DcMotor right_back, DcMotor left_front, DcMotor left_back){
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    private void detectEnd(Pose2d currentPose) {
        reachedDestination = PurePursuitMathFunctions.reachedDestination(currentPose, pathEnd, reachedDestination, duration, PPPath.path1.get(PPPath.path1.size()-1).t);
        if (reachedDestination == PurePursuitMathFunctions.ReachedDestination.TRUE) {
            exportTelemetryData();
            this.stop();
        }
    }

    private void exportTelemetryData() {
        if (xHist.size() > 0) {
            String tString = "";
            String xString = "";
            String yString = "";
            String hString = "";
            String vString = "";
            String aString = "";
            String angVString = "";
            String angAString = "";
//                BufferedWriter myWriter = new BufferedWriter(new FileWriter("telemetry.txt"));
                RobotLogger.dd(TAG, "xHist.size(): " + xHist.size());
                RobotLogger.dd(TAG, "yHist.size(): " + yHist.size());
                RobotLogger.dd(TAG, "hHist.size(): " + hHist.size());
                RobotLogger.dd(TAG, "vHist.size(): " + vHist.size());
                RobotLogger.dd(TAG, "aHist.size(): " + aHist.size());
                RobotLogger.dd(TAG, "angVHist.size(): " + angVHist.size());
                RobotLogger.dd(TAG, "angAHist.size(): " + angAHist.size());

            for (int i=0; i<xHist.size(); i++) {
                tString += tHist.get(i) + ",";
                xString += xHist.get(i) + ",";
                yString += yHist.get(i) + ",";
                hString += hHist.get(i) + ",";
                vString += vHist.get(i) + ",";
                aString += aHist.get(i) + ",";
                angVString += angVHist.get(i) + ",";
                angAString += angAHist.get(i) + ",";
            }
//                myWriter.write("x:" + xString + "\n");
//                myWriter.write("y:" + yString + "\n");
//                myWriter.write("h:" + hString + "\n");
//                myWriter.write("v:" + vString + "\n");
//                myWriter.write("a:" + aString + "\n");
//                myWriter.write("angV:" + angVString + "\n");
//                myWriter.write("angA:" + angAString + "\n");
//                myWriter.close();
            String allTelemetry = "t:" + tString + "\n" + "x:" + xString + "\n" + "y:" + yString + "\n" + "h:" + hString + "\n" + "v:" + vString + "\n" + "a:" + aString + "\n" + "angV:" + angVString + "\n" + "angA:" + angAString + "\n";

            String fullOutput = MPString + "\n\n" + "*" + "\n\n" + allTelemetry;
            ReadWriteFile.writeFile(logFile, fullOutput);

//            String separator = "*";
//            ReadWriteFile.writeFile(logFile, separator);  //* char defines the split between target data and real data
//            ReadWriteFile.writeFile(logFile, MPString);

            RobotLogger.dd(TAG, "Successfully wrote to telemetry file.");
        }
    }
}
