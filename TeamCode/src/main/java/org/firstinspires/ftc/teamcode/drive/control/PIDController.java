package org.firstinspires.ftc.teamcode.drive.control;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.xStartPower;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.yStartPower;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hStartPower;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.xAccel;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.yAccel;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hAccel;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kVM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kAM;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngVel;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngAccel;

import java.util.ArrayList;

public class PIDController {
    final String TAG = "PIDController";

    PurePursuitPath PPPath;
    ArrayList<Pose2d> poseHistoryLocal = new ArrayList<>();
    ArrayList<Pose2d> errorHistoryPercents = new ArrayList<>();
    ArrayList<Pose2d> errorHistory = new ArrayList<>();
    Pose2d startPose;
    Pose2d target;
    ArrayList<Double> outputs;
    PurePursuitPathPoint nearestPoint;
    int nearestPointIndex;
    int targetIndex;

    double startingErrorX;
    double startingErrorY;
    double startingErrorHeading;
    double currentErrorX;
    double currentErrorY;
    double currentErrorHeading;
    double currentErrorPercentX;
    double currentErrorPercentY;
    double currentErrorPercentHeading;
    double jerkControlMultiplier;

    double xP;
    double xI;
    double xD;
    double yP;
    double yI;
    double yD;
    double hP;
    double hI;
    double hD;

    double PIDFRatio = 0.35;     //Percentage of power supplied by feedforward

    public PIDController(Pose2d robotPose, Pose2d target,  double xP, double xI, double xD, double yP, double yI, double yD, double hP, double hI, double hD) {
        startPose = robotPose;
        this.target = target;
        this.xP = xP;
        this.xI = xI;
        this.xD = xD;
        this.yP = yP;
        this.yI = yI;
        this.yD = yD;
        this.hP = hP;
        this.hI = hI;
        this.hD = hD;

        startingErrorX = startPose.getX() - target.getX();
        startingErrorY = startPose.getY() - target.getY();
        startingErrorHeading = startPose.getHeading() - target.getHeading();
        //Must ensure starting errors are not 0 to avoid NaN
        startingErrorX = ensureNonZero(startingErrorX);
        startingErrorY = ensureNonZero(startingErrorY);
        startingErrorHeading = ensureNonZero(normalizeHeading(startingErrorHeading));
        RobotLogger.dd(TAG, "Controller startingError: " + startingErrorX + ", " + startingErrorY + ", " + startingErrorHeading);
    }

    public PIDController(Pose2d robotPose, Pose2d target, int targetIndex, PurePursuitPath PPPath, double xP, double xI, double xD, double yP, double yI, double yD, double hP, double hI, double hD) {
        startPose = robotPose;
        this.target = target;
        this.xP = xP;
        this.xI = xI;
        this.xD = xD;
        this.yP = yP;
        this.yI = yI;
        this.yD = yD;
        this.hP = hP;
        this.hI = hI;
        this.hD = hD;
        this.PPPath = PPPath;
        this.targetIndex = targetIndex;

        startingErrorX = startPose.getX() - target.getX();
        startingErrorY = startPose.getY() - target.getY();
        startingErrorHeading = startPose.getHeading() - target.getHeading();
        //Must ensure starting errors are not 0 to avoid NaN
        startingErrorX = ensureNonZero(startingErrorX);
        startingErrorY = ensureNonZero(startingErrorY);
        startingErrorHeading = ensureNonZero(normalizeHeading(startingErrorHeading));
        RobotLogger.dd(TAG, "Controller startingError: " + startingErrorX + ", " + startingErrorY + ", " + startingErrorHeading);
    }


    public ArrayList<Double> update(Pose2d robotPose, int prevLoopTime) {
        poseHistoryLocal.add(robotPose);
        outputs = new ArrayList<>();
        outputs.clear();
//        RobotLogger.dd(TAG, "robotPose: " + robotPose.toString());
//        RobotLogger.dd(TAG, "target: " + "(" + target.getX() + ", " + target.getY() + ", " + target.getHeading() + ")");
        currentErrorX = robotPose.getX() - target.getX();
        currentErrorY = robotPose.getY() - target.getY();
        currentErrorHeading = normalizeHeading(robotPose.getHeading() - target.getHeading());
        RobotLogger.dd(TAG, "currentErrorHeading: " + currentErrorHeading);
        errorHistory.add(new Pose2d(currentErrorX, currentErrorY, currentErrorHeading));

        //Percent errors always positive! Sign is checked later.
        //(Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY)) = the larger of the two errors
        currentErrorPercentX = Math.abs(currentErrorX / (Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY))));
        currentErrorPercentY = Math.abs(currentErrorY / (Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY))));
        //Ensure that startingErrorHeading isn't close to 0, or currentErrorPercentHeading can easily exceed 1.0
        startingErrorHeading = Math.abs(startingErrorHeading);
        if (Math.toDegrees(startingErrorHeading) < 15) {
            startingErrorHeading = Math.toRadians(15.5 - ((1 * 7)/((0.6 * Math.toDegrees(startingErrorHeading)) + 1)));
        }
        RobotLogger.dd(TAG, "startingErrorHeading after: " + startingErrorHeading);
        currentErrorPercentHeading = Math.abs(currentErrorHeading) / startingErrorHeading;
        errorHistoryPercents.add(new Pose2d(currentErrorPercentX, currentErrorPercentY, currentErrorPercentHeading));

//        RobotLogger.dd(TAG, "xCurrentError: " + errorHistory.get(errorHistory.size() - 1).getX());
//        RobotLogger.dd(TAG, "yCurrentError: " + errorHistory.get(errorHistory.size() - 1).getY());
//        RobotLogger.dd(TAG, "hCurrentError: " + errorHistory.get(errorHistory.size() - 1).getHeading());
        RobotLogger.dd(TAG, "xPercentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX());
        RobotLogger.dd(TAG, "yPercentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY());
        RobotLogger.dd(TAG, "hPercentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading());


        double smallestDiff = 99999.9;
        int smallestDiffIndex = 0;
        double diff;
        //Setting the bounds for all the points in the path to check, avoiding index out of bounds error
        int minBound;
        if (targetIndex - 150 < 0) {
            minBound = 0;
        }
        else {
            minBound = targetIndex - 150;
        }
        int maxBound;
        if (targetIndex + 75 > PPPath.path1.size() - 1) {
            maxBound = PPPath.path1.size() - 1;
        }
        else {
            maxBound = targetIndex + 75;
        }
        for (int i=minBound; i<maxBound; i++) {    //Searching range of points around target to find nearestPoint
            diff = Math.hypot(robotPose.getX() - PPPath.path1.get(i).x, robotPose.getY() - PPPath.path1.get(i).y);
//            diff = Math.sqrt(Math.pow(robotPose.getX() - PPPath.path1.get(i).x, 2) + Math.pow(robotPose.getY() - PPPath.path1.get(i).y, 2));
            if (diff <= smallestDiff) {
                smallestDiff = diff;
                smallestDiffIndex = i;
            }
//            else {
//                break;
//            }
        }
        nearestPoint = PPPath.path1.get(smallestDiffIndex);
        nearestPointIndex = smallestDiffIndex;


        //proportional calculator: outputs present error * xP
        double pXOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = xStartPower + xAccel * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX()));
        if (jerkControlMultiplier > 1) {
            jerkControlMultiplier = 1;
        }
        if (errorHistory.get(errorHistory.size() - 1).getX() >= 0.0) {
            pXOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX() * xP);
        }
        else {
            pXOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX() * xP);
        }

//        if (errorHistory.get(errorHistory.size() - 1).getX() >= 0) {
//            pXOutput = -xP * errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX();
//        }
//        else {
//            pXOutput = xP * errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX();
//        }


        //integral calculator: outputs integral of all previous xErrors * xI
        Double iXOutput = 0.0;
        for (Pose2d errorVector : errorHistoryPercents) {
            iXOutput += errorVector.getX();
        }
        if (errorHistory.get(errorHistory.size() - 1).getX() >= 0.0) {
            iXOutput = -iXOutput * (xI/400);    //Dividing by large number so the inputted xI value can be a comprehensibly large number
        }
        else {
            iXOutput = iXOutput * (xI/400);     //Dividing by large number so the inputted xI value can be a comprehensibly large number
        }

        //derivative calculator: outputs approximate derivative (using difference quotient) * xD
        double dXOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dXOutput = 0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1).getX() >= 0.0) {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dXOutput = -(xD * 40) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getX())) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dXOutput = (xD * 40) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getX())) / prevLoopTime;
            }
        }



        //proportional calculator: outputs present error * yP
        double pYOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = yStartPower + yAccel * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY()));
        if (jerkControlMultiplier > 1) {
            jerkControlMultiplier = 1;
        }
        if (errorHistory.get(errorHistory.size() - 1).getY() >= 0.0) {
            pYOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY() * yP);
        }
        else {
            pYOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY() * yP);
        }

        //integral calculator: outputs integral of all previous yErrors * yi
        double iYOutput = 0.0;
        for (Pose2d errorVector : errorHistoryPercents) {
            iYOutput += errorVector.getY();
        }
        if (errorHistory.get(errorHistory.size() - 1).getY() >= 0.0) {
            iYOutput = -iYOutput * (yI/500);    //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }
        else {
            iYOutput = iYOutput * (yI/500);     //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }


        //derivative calculator: outputs approximate derivative (using difference quotient) * yd
        double dYOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dYOutput = 0.0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1).getY() >= 0.0) {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dYOutput = -(yD * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getY())) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dYOutput = (yD * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getY())) / prevLoopTime;
            }
        }



        //proportional calculator: outputs present error * hP
        double pHOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = hStartPower + hAccel * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading()));
        if (jerkControlMultiplier > 0.5) {    //Set max value
            jerkControlMultiplier = 0.5;
        }
        if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
            //Multiplying by a constant so the inputted hP value can be around 1.0
            pHOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() * 5 * hP);
        }
        else {
            //Multiplying by a constant so the inputted hP value can be around 1.0
            pHOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() * 5 * hP);
        }

        //integral calculator: outputs integral of all previous hErrors * hi
        double iHOutput = 0.0;
        for (Pose2d errorVector : errorHistoryPercents) {
            iHOutput += errorVector.getHeading();
        }
        if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
            iHOutput = -iHOutput * (hI/300);    //Dividing by large number so the inputted hI value can be a comprehensibly large number
        }
        else {
            iHOutput = iHOutput * (hI/300);     //Dividing by large number so the inputted hI value can be a comprehensibly large number
        }

        //derivative calculator: outputs approximate derivative (using difference quotient) * hd
        double dHOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dHOutput = 0.0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
                //Multiplying by a constant so the inputted hD value can be around 1.0
                dHOutput = -(hD * 18) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getHeading())) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted hD value can be around 1.0
                dHOutput = (hD * 18) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getHeading())) / prevLoopTime;
            }
        }



        //Weights of PID vs Feedforward is (1.0-PIDFRatio) and PIDFRatio
        /*
        double kVRatio = kVM/(kVM+kAM);
        double kARatio = kAM/(kVM+kAM);
        double feedForwardXYOutput = (kVRatio*PIDFRatio*nearestPoint.velocity)/maxV + (kARatio*PIDFRatio*nearestPoint.acceleration)/maxA;
        double feedForwardHOutput = (kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV + (kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA;
//        double directionOfMotion = Math.atan2(-robotPose.getY()+target.getY(), -robotPose.getX()+target.getX());    //Returns between -pi and pi, on a standard cartesion system (north is 90)

        //Returns between -pi and pi, on robot coordinate system (north is 0, left is negative)
        double directionOfMotion = -Math.atan2(target.getY()-robotPose.getY(), target.getX()-robotPose.getX()) + Math.PI/2;
        if (directionOfMotion > Math.PI) {
            directionOfMotion -= 2*Math.PI;
        }

        RobotLogger.dd(TAG, "directionOfMotion: " + directionOfMotion);
        //Set the correct sign for the feedforward output using signum()
//        double fXOutput = feedForwardXYOutput * Math.sin(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorX);
//        double fYOutput = feedForwardXYOutput * Math.cos(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorY);

        //sin and cos only function with a cartesian heading system, so we need to translate from robot heading system to cartesian by multiplying (robotCartesianHeading - directionOfMotion) by -1
        double fXOutput = feedForwardXYOutput * Math.cos(robotPose.getHeading() + directionOfMotion);
        double fYOutput = feedForwardXYOutput * Math.sin(robotPose.getHeading() + directionOfMotion);

//        double fXOutput = feedForwardXYOutput * Math.sin(-(robotPose.getHeading() - directionOfMotion));
//        double fYOutput = feedForwardXYOutput * Math.cos(-(robotPose.getHeading() - directionOfMotion));
        double fHOutput = feedForwardHOutput;
        */

        ArrayList<Double> feedforwardOutputs = getFeedForwardPowersV1(robotPose);
        double fXOutput = feedforwardOutputs.get(0);
        double fYOutput = feedforwardOutputs.get(1);
        double fHOutput = feedforwardOutputs.get(2);
        double directionOfMotion = feedforwardOutputs.get(3);
        double kVRatio = feedforwardOutputs.get(4);
        double kARatio = feedforwardOutputs.get(5);

        double xNetOutput = (1.0-PIDFRatio)*(pXOutput + iXOutput + dXOutput) + fXOutput;    //PIDFRatio for feedforward output is factored in in the getFeedForwardPowers Function
        double yNetOutput = (1.0-PIDFRatio)*(pYOutput + iYOutput + dYOutput) + fYOutput;    //PIDFRatio for feedforward output is factored in in the getFeedForwardPowers Function
        double hNetOutput = (1.0-PIDFRatio)*(pHOutput + iHOutput + dHOutput) + fHOutput;    //PIDFRatio for feedforward output is factored in in the getFeedForwardPowers Function

        RobotLogger.dd(TAG, "nearestPoint: " + nearestPoint.toString());
        RobotLogger.dd(TAG, "xPIDF: " + pXOutput + ", " + iXOutput + ", " + dXOutput + ", " + fXOutput);
        RobotLogger.dd(TAG, "yPIDF: " + pYOutput + ", " + iYOutput + ", " + dYOutput + ", " + fYOutput);
        RobotLogger.dd(TAG, "hPIDF: " + pHOutput + ", " + iHOutput + ", " + dHOutput + ", " + fHOutput);
        double targetXV = nearestPoint.velocity * Math.sin(-(robotPose.getHeading() - directionOfMotion)) * -Math.signum(startingErrorX);
        double targetYV = nearestPoint.velocity * Math.cos(-(robotPose.getHeading() - directionOfMotion)) * -Math.signum(startingErrorX);
        RobotLogger.dd(TAG, "targetVelocity / maxV: " + PPPath.path1.get(nearestPointIndex).velocity + " / " + maxV);
//        RobotLogger.dd(TAG, "targetXVelocity: " + targetXV);
//        RobotLogger.dd(TAG, "targetYVelocity: " + targetYV);
        RobotLogger.dd(TAG, "(kVRatio*PIDFRatio*nearestPoint.velocity)/maxV: " + ((kVRatio*PIDFRatio*nearestPoint.velocity)/maxV));
        RobotLogger.dd(TAG, "(kARatio*PIDFRatio*nearestPoint.acceleration)/maxA: " + ((kARatio*PIDFRatio*nearestPoint.acceleration)/maxA));
        RobotLogger.dd(TAG, "(kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV: " + ((kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV));
        RobotLogger.dd(TAG, "(kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA: " + ((kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA));



        //STABLE
//        double xNetOutput = pXOutput + iXOutput + dXOutput;
//        double yNetOutput = pYOutput + iYOutput + dYOutput;
//        double hNetOutput = pHOutput + iHOutput + dHOutput;

        //xNet and yNet are local to X and Y of robot
        ArrayList<Double> powers = vectorToPowersV4(xNetOutput, yNetOutput, hNetOutput);    //V4 should be the latest version

        return powers;
    }

    public ArrayList<Double> vectorToPowersV1(double vectorX, double vectorY, double rotationVelocity) {
        //vectorX and vectorY are local to X and Y of robot
        double magnitude = Math.sqrt(Math.pow(vectorY, 2) + Math.pow(vectorX, 2));

        RobotLogger.dd(TAG, "vectorY: " + vectorY);
        RobotLogger.dd(TAG, "vectorX: " + vectorX);
        //                              opp      adj
        double direction = Math.atan2(vectorX, vectorY);

        double sin = Math.sin(direction + Math.PI / 4.0);
        double cos = Math.cos(direction + Math.PI / 4.0);

        double largerComponent = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= largerComponent;
        cos /= largerComponent;

        double powerFL = magnitude * sin + rotationVelocity;
        double powerFR = magnitude * cos - rotationVelocity;
        double powerBR = magnitude * sin - rotationVelocity;
        double powerBL = magnitude * cos + rotationVelocity;

        double scaleFactor = limitPower(1.0, powerFL, powerFR, powerBR, powerBL);

        ArrayList<Double> powers = new ArrayList<>();
//        Log.d(TAG, "ScaleFactor: " + scaleFactor);
        powers.add(powerFL / scaleFactor);
        powers.add(powerBL / scaleFactor);
        powers.add(powerBR / scaleFactor);
        powers.add(powerFR / scaleFactor);
        return powers;
    }

    public ArrayList<Double> vectorToPowersV2(double vectorX, double vectorY, double rotationVelocity) {
        double yPowerFL = vectorY;
        double yPowerFR = vectorY;
        double yPowerBR = vectorY;
        double yPowerBL = vectorY;

        double xPowerFL = vectorX;
        double xPowerFR = -vectorX;
        double xPowerBR = vectorX;
        double xPowerBL = -vectorX;

        double hPowerFL = rotationVelocity;
        double hPowerFR = -rotationVelocity;
        double hPowerBR = -rotationVelocity;
        double hPowerBL = rotationVelocity;

        double powerFL = xPowerFL + yPowerFL + hPowerFL;
        double powerFR = xPowerFR + yPowerFR + hPowerFR;
        double powerBR = xPowerBR + yPowerBR + hPowerBR;
        double powerBL = xPowerBL + yPowerBL + hPowerBL;

        double scaleFactor = limitPower(1.0, powerFL, powerFR, powerBR, powerBL);

        ArrayList<Double> powers = new ArrayList<>();
//        Log.d(TAG, "ScaleFactor: " + scaleFactor);
        powers.add(powerFL / scaleFactor);
        powers.add(powerBL / scaleFactor);
        powers.add(powerBR / scaleFactor);
        powers.add(powerFR / scaleFactor);
        return powers;
    }

    public ArrayList<Double> vectorToPowersV3(double vectorXGlobal, double vectorYGlobal, double rotationVelocity) {
        double currentHeading = poseHistoryLocal.get(poseHistoryLocal.size() - 1).getHeading();
//        RobotLogger.dd(TAG, "currentHeading: " + currentHeading);
//        double directionOfMotion = Math.atan2(target.getY() - startPose.getX(), target.getX() - startPose.getY());
//        RobotLogger.dd(TAG, "directionOfMotion: " + directionOfMotion);
//        double headingMotionDifference = currentHeading - directionOfMotion;
//        RobotLogger.dd(TAG, "headingMotionDifference: " + headingMotionDifference);
        double heading90Difference = (Math.PI/2) - currentHeading;
//        RobotLogger.dd(TAG, "heading90Difference: " + heading90Difference);

        RobotLogger.dd(TAG, "vectorXGlobal: " + vectorXGlobal);
        RobotLogger.dd(TAG, "vectorYGlobal: " + vectorYGlobal);
//        ArrayList<Double> localVector = vectorGlobalToLocal(vectorXGlobal, vectorYGlobal, -headingMotionDifference);
        ArrayList<Double> localVector = vectorGlobalToLocal(vectorXGlobal, vectorYGlobal, heading90Difference);
        double vectorX = localVector.get(0);
        double vectorY = localVector.get(1);
        RobotLogger.dd(TAG, "vectorX: " + vectorX);
        RobotLogger.dd(TAG, "vectorY: " + vectorY);

        //vectorX and vectorY are local to X and Y of robot
        double magnitude = Math.sqrt(Math.pow(vectorY, 2) + Math.pow(vectorX, 2));
        //                              opp      adj
        double direction = Math.atan2(vectorX, vectorY);

        double sin = Math.sin(direction + Math.PI / 4.0);
        double cos = Math.cos(direction + Math.PI / 4.0);

        double largerComponent = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= largerComponent;
        cos /= largerComponent;

        double powerFL = magnitude * sin + rotationVelocity;
        double powerFR = magnitude * cos - rotationVelocity;
        double powerBR = magnitude * sin - rotationVelocity;
        double powerBL = magnitude * cos + rotationVelocity;

        double scaleFactor = limitPower(1.0, powerFL, powerFR, powerBR, powerBL);

        ArrayList<Double> powers = new ArrayList<>();
//        Log.d(TAG, "ScaleFactor: " + scaleFactor);
        powers.add(powerFL / scaleFactor);
        powers.add(powerBL / scaleFactor);
        powers.add(powerBR / scaleFactor);
        powers.add(powerFR / scaleFactor);
        return powers;
    }

    public ArrayList<Double> vectorToPowersV4(double vectorXGlobal, double vectorYGlobal, double rotationVelocity) {
        double currentHeading = poseHistoryLocal.get(poseHistoryLocal.size() - 1).getHeading();
        RobotLogger.dd(TAG, "vectorXGlobal: " + vectorXGlobal);
        RobotLogger.dd(TAG, "vectorYGlobal: " + vectorYGlobal);
        ArrayList<Double> localVector = vectorGlobalToLocal(vectorXGlobal, vectorYGlobal, currentHeading);
        double vectorX = localVector.get(0);
        double vectorY = localVector.get(1);
        RobotLogger.dd(TAG, "vectorX: " + vectorX);
        RobotLogger.dd(TAG, "vectorY: " + vectorY);
        RobotLogger.dd(TAG, "rotationVelocity: " + rotationVelocity);

        //vectorX and vectorY are local to X and Y of robot
        double magnitude = Math.sqrt(Math.pow(vectorY, 2) + Math.pow(vectorX, 2));
        //                              opp      adj
        double direction = Math.atan2(vectorX, vectorY);

        double sin = Math.sin(direction + Math.PI / 4.0);
        double cos = Math.cos(direction + Math.PI / 4.0);

        double largerComponent = Math.max(Math.abs(sin), Math.abs(cos));
        sin /= largerComponent;
        cos /= largerComponent;

        double powerFL = magnitude * sin + rotationVelocity;
        double powerFR = magnitude * cos - rotationVelocity;
        double powerBR = magnitude * sin - rotationVelocity;
        double powerBL = magnitude * cos + rotationVelocity;

        double scaleFactor = limitPower(1.0, powerFL, powerFR, powerBR, powerBL);

        ArrayList<Double> powers = new ArrayList<>();
//        Log.d(TAG, "ScaleFactor: " + scaleFactor);
        powers.add(powerFL / scaleFactor);
        powers.add(powerBL / scaleFactor);
        powers.add(powerBR / scaleFactor);
        powers.add(powerFR / scaleFactor);
        return powers;
    }

    public ArrayList<Double> getFeedForwardPowersV1(Pose2d robotPose) {
        double kVRatio = kVM/(kVM+kAM);
        double kARatio = kAM/(kVM+kAM);
        double feedForwardXYOutput = (kVRatio*PIDFRatio*PPPath.path1.get(nearestPointIndex).velocity)/maxV + (kARatio*PIDFRatio*PPPath.path1.get(nearestPointIndex).acceleration)/maxA + kStatic;
        double feedForwardHOutput = (kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV + (kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA;
//        double feedForwardHOutput = ((kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV + (kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA + kStatic) * -Math.signum(startingErrorHeading);
//        double feedForwardHOutput = 0.0;
//        double directionOfMotion = Math.atan2(-robotPose.getY()+target.getY(), -robotPose.getX()+target.getX());    //Returns between -pi and pi, on a standard cartesion system (north is 90)

        //Returns between -pi and pi, on robot coordinate system (north is 0, left is negative)
        double directionOfMotion = -Math.atan2(target.getY()-robotPose.getY(), target.getX()-robotPose.getX()) + Math.PI/2;
        if (directionOfMotion > Math.PI) {
            directionOfMotion -= 2*Math.PI;
        }

        RobotLogger.dd(TAG, "directionOfMotion: " + Math.toDegrees(directionOfMotion));
        //Set the correct sign for the feedforward output using signum()
//        double fXOutput = feedForwardXYOutput * Math.sin(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorX);
//        double fYOutput = feedForwardXYOutput * Math.cos(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorY);

        //sin and cos only function with a cartesian heading system, so we need to translate from robot heading system to cartesian by multiplying (robotCartesianHeading - directionOfMotion) by -1
        double fXOutput = feedForwardXYOutput * Math.sin(directionOfMotion - robotPose.getHeading());
        double fYOutput = feedForwardXYOutput * Math.cos(directionOfMotion - robotPose.getHeading());

        ArrayList<Double> outputs = new ArrayList<>();
        outputs.add(fXOutput);
        outputs.add(fYOutput);
        outputs.add(feedForwardHOutput);
        outputs.add(directionOfMotion);
        outputs.add(kVRatio);
        outputs.add(kARatio);
        return outputs;
    }

    public ArrayList<Double> getFeedForwardPowersV2(Pose2d robotPose) {
        double kVRatio = kVM/(kVM+kAM);
        double kARatio = kAM/(kVM+kAM);
        double feedForwardXYOutput = (kVRatio*PIDFRatio*nearestPoint.velocity)/maxV + (kARatio*PIDFRatio*nearestPoint.acceleration)/maxA;
        double feedForwardHOutput = (kVRatio*PIDFRatio*nearestPoint.angVelocity)/maxAngV + (kARatio*PIDFRatio*nearestPoint.angAcceleration)/maxAngA;
//        double directionOfMotion = Math.atan2(-robotPose.getY()+target.getY(), -robotPose.getX()+target.getX());    //Returns between -pi and pi, on a standard cartesion system (north is 90)

        //Returns between -pi and pi, on robot coordinate system (north is 0, left is negative)
        double directionOfMotion = Math.atan2(target.getY()-robotPose.getY(), target.getX()-robotPose.getX()) - Math.PI/2;
        if (directionOfMotion < -Math.PI) {
            directionOfMotion += 2*Math.PI;
        }

        RobotLogger.dd(TAG, "directionOfMotion: " + directionOfMotion);
        //Set the correct sign for the feedforward output using signum()
//        double fXOutput = feedForwardXYOutput * Math.sin(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorX);
//        double fYOutput = feedForwardXYOutput * Math.cos(-(robotPose.getHeading() + 90) - directionOfMotion) * -Math.signum(startingErrorY);

        //sin and cos only function with a cartesian heading system, so we need to translate from robot heading system to cartesian by multiplying (robotCartesianHeading - directionOfMotion) by -1
        double fXOutput = feedForwardXYOutput * Math.cos(directionOfMotion - robotPose.getHeading());
        double fYOutput = feedForwardXYOutput * Math.sin(directionOfMotion - robotPose.getHeading());

        ArrayList<Double> outputs = new ArrayList<>();
        outputs.add(fXOutput);
        outputs.add(fYOutput);
        outputs.add(feedForwardHOutput);
        outputs.add(directionOfMotion);
        outputs.add(kVRatio);
        outputs.add(kARatio);
        return outputs;
    }

    public ArrayList<Double> vectorGlobalToLocal(double vectorX, double vectorY, double globalHeading) {
        double localVectorX = vectorX * Math.cos(-globalHeading) + vectorY * Math.sin(-globalHeading);
        double localVectorY = - vectorX * Math.sin(-globalHeading) + vectorY * Math.cos(-globalHeading);

        ArrayList<Double> localVector = new ArrayList<>();
        localVector.add(localVectorX);
        localVector.add(localVectorY);
        return localVector;
    }

    public double limitPower(double max, double p1, double p2, double p3, double p4) {
        /*
         * Ensure all motors powers are under "max"
         */
        double largest = Math.abs(p1);
        if (Math.abs(p2) > largest) {
            largest = Math.abs(p2);
        }
        if (Math.abs(p3) > largest) {
            largest = Math.abs(p3);
        }
        if (Math.abs(p4) > largest) {
            largest = Math.abs(p4);
        }

        if (largest > max) {
            return largest;
        }
        return max;
    }

    public double ensureNonZero(double error) {
        if (Math.abs(error) < 0.01) {
            return 0.01;
        }
        else {
            return error;
        }
    }

    public static double normalizeHeading(double angle) {
        //Ensures angle turn is between 180 and -180
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}
