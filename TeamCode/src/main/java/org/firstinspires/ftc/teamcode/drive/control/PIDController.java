package org.firstinspires.ftc.teamcode.drive.control;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;

public class PIDController {
    final String TAG = "PIDController";

    //    ArrayList<Pose2d> poseHistoryLocal;
    ArrayList<Pose2d> errorHistoryPercents = new ArrayList<>();
    ArrayList<Pose2d> errorHistory = new ArrayList<>();
    Pose2d startPose;
    Pose2d target;
    ArrayList<Double> outputs;
    double distance;

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
        RobotLogger.dd(TAG, "startingErrorX start" + startingErrorX);
        startingErrorX = ensureNonZero(startingErrorX);
        RobotLogger.dd(TAG, "startingErrorX after ensure nonzero" + startingErrorX);
        startingErrorY = ensureNonZero(startingErrorY);
        startingErrorHeading = ensureNonZero(startingErrorHeading);
        RobotLogger.dd(TAG, "Controller startingError: " + startingErrorX + ", " + startingErrorY + ", " + startingErrorHeading);
    }


    public ArrayList<Double> update(Pose2d robotPose, int prevLoopTime) {
        outputs = new ArrayList<>();
        outputs.clear();
//        RobotLogger.dd(TAG, "robotPose: " + robotPose.toString());
        RobotLogger.dd(TAG, "target: " + target.toString());
        currentErrorX = robotPose.getX() - target.getX();
        currentErrorY = robotPose.getY() - target.getY();
        currentErrorHeading = robotPose.getHeading() - target.getHeading();
        errorHistory.add(new Pose2d(currentErrorX, currentErrorY, currentErrorHeading));

//        currentErrorPercentX = errorHistory.get(errorHistory.size() - 1).getX() / startingErrorX;
//        currentErrorPercentY = errorHistory.get(errorHistory.size() - 1).getY() / startingErrorY;
//        currentErrorPercentHeading = errorHistory.get(errorHistory.size() - 1).getHeading() / startingErrorHeading;

        //(Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY)) = the larger of the two errors
        currentErrorPercentX = Math.abs(currentErrorX / (Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY))));
        currentErrorPercentY = Math.abs(currentErrorY / (Math.max(Math.abs(startingErrorX), Math.abs(startingErrorY))));
        currentErrorPercentHeading = currentErrorHeading / startingErrorHeading;
        errorHistoryPercents.add(new Pose2d(currentErrorPercentX, currentErrorPercentY, currentErrorPercentHeading));

        RobotLogger.dd(TAG, "xCurrentError: " + errorHistory.get(errorHistory.size() - 1).getX());
        RobotLogger.dd(TAG, "yCurrentError: " + errorHistory.get(errorHistory.size() - 1).getY());
        RobotLogger.dd(TAG, "xPercentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1).getX());
        RobotLogger.dd(TAG, "yPercentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1).getY());



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

        //integral calculator: outputs integral of all previous xErrors * xKi
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


        //derivative calculator: outputs approximate derivative (using difference quotient) * yKd
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

        double xNetOutput = pXOutput + iXOutput + dXOutput;
        double yNetOutput = pYOutput + iYOutput + dYOutput;

        ArrayList<Double> localVector = vectorGlobalToLocal(xNetOutput, yNetOutput, robotPose.getHeading());
        double localVectorX = localVector.get(0);
        double localVectorY = localVector.get(1);

        //vectorX and vectorY are local to X and Y of robot
//        ArrayList<Double> powers = vectorToPowersV1(localVectorX, localVectorY, 0.0);
        ArrayList<Double> powers = vectorToPowersV1(xNetOutput, yNetOutput, 0.0);
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
        Log.d(TAG, "ScaleFactor: " + scaleFactor);
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
}
