package org.firstinspires.ftc.teamcode.drive.control;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;

public class PIDController {
    final String TAG = "PIDController";

    ArrayList<Pose2d> poseHistoryLocal = new ArrayList<>();
    ArrayList<Pose2d> errorHistoryPercents = new ArrayList<>();
    ArrayList<Pose2d> errorHistory = new ArrayList<>();
    Pose2d startPose;
    Pose2d target;
    ArrayList<Double> outputs;
    double distance;    //currently a useless variable

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

    public PIDController(Pose2d robotPose, Pose2d target, double distance, double xP, double xI, double xD, double yP, double yI, double yD, double hP, double hI, double hD) {
        startPose = robotPose;
        this.target = target;
        this.distance = distance;
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


    public ArrayList<Double> update(Pose2d robotPose, int prevLoopTime) {
        poseHistoryLocal.add(robotPose);
        outputs = new ArrayList<>();
        outputs.clear();
//        RobotLogger.dd(TAG, "robotPose: " + robotPose.toString());
        RobotLogger.dd(TAG, "target: " + "(" + target.getX() + ", " + target.getY() + ", " + target.getHeading() + ")");
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



        //proportional calculator: outputs present error * xP
        double pXOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = 0.75 + 2 * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX()));
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
        RobotLogger.dd(TAG, "xPID: " + pXOutput + ", " + iXOutput + ", " + dXOutput);



        //proportional calculator: outputs present error * yP
        double pYOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = 0.90 + 4 * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY()));
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
        RobotLogger.dd(TAG, "yPID: " + pYOutput + ", " + iYOutput + ", " + dYOutput);



        //proportional calculator: outputs present error * hP
        double pHOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = 0.3 + 6 * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading()));
        if (jerkControlMultiplier > 0.5) {    //Set max value
            jerkControlMultiplier = 0.5;
        }
        if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
            //Multiplying by a constant so the inputted hP value can be around 1.0
            pHOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() * 5 * hP);
        }
        else {
            //Multiplying by a constant so the inputted hP value can be around 1.0
            pHOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() * 5 * hP);
        }

        //integral calculator: outputs integral of all previous hErrors * hi
        double iHOutput = 0.0;
        for (Pose2d errorVector : errorHistoryPercents) {
            iHOutput += errorVector.getHeading();
        }
        if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
            iHOutput = iHOutput * (hI/300);    //Dividing by large number so the inputted hI value can be a comprehensibly large number
        }
        else {
            iHOutput = -iHOutput * (hI/300);     //Dividing by large number so the inputted hI value can be a comprehensibly large number
        }

        //derivative calculator: outputs approximate derivative (using difference quotient) * hd
        double dHOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dHOutput = 0.0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1).getHeading() >= 0.0) {
                //Multiplying by a constant so the inputted hD value can be around 1.0
                dHOutput = (hD * 18) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getHeading())) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted hD value can be around 1.0
                dHOutput = -(hD * 18) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1).getHeading() - errorHistoryPercents.get(errorHistoryPercents.size() - 2).getHeading())) / prevLoopTime;
            }
        }
        RobotLogger.dd(TAG, "hPID: " + pHOutput + ", " + iHOutput + ", " + dHOutput);



        double xNetOutput = pXOutput + iXOutput + dXOutput;
        double yNetOutput = pYOutput + iYOutput + dYOutput;
        double hNetOutput = pHOutput + iHOutput + dHOutput;

//        ArrayList<Double> localVector = vectorGlobalToLocal(xNetOutput, yNetOutput, robotPose.getHeading());
//        double localVectorX = localVector.get(0);
//        double localVectorY = localVector.get(1);

        //vectorX and vectorY are local to X and Y of robot
        //v3 is stable in the old configuration
        ArrayList<Double> powers = vectorToPowersV4(xNetOutput, yNetOutput, hNetOutput);
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
        ArrayList<Double> localVector = vectorGlobalToLocal(vectorXGlobal, vectorYGlobal, -currentHeading);
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

    public ArrayList<Double> vectorToPowersV5(double vectorXGlobal, double vectorYGlobal, double rotationVelocity) {
        double currentHeading = poseHistoryLocal.get(poseHistoryLocal.size() - 1).getHeading();
        RobotLogger.dd(TAG, "vectorXGlobal: " + vectorXGlobal);
        RobotLogger.dd(TAG, "vectorYGlobal: " + vectorYGlobal);
        ArrayList<Double> localVector = vectorGlobalToLocal(vectorXGlobal, vectorYGlobal, -currentHeading);
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
