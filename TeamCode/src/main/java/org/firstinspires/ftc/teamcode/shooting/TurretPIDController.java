package org.firstinspires.ftc.teamcode.shooting;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.RobotLogger;
import java.util.ArrayList;

public class TurretPIDController {
    final String TAG = "PIDController";
    final int ticksPerRev = 384;
    //Set the bounds of the turret rotation so wiring doesn't break
    final double leftBound = 0;
    final double rightBound = 360;

    ArrayList<Double> positionHistoryLocal = new ArrayList<>();
    ArrayList<Double> errorHistoryPercents = new ArrayList<>();
    ArrayList<Double> errorHistory = new ArrayList<>();

    Double startPosition;
    Double target;
    ArrayList<Double> outputs;

    double startingError;
    double currentError;
    double currentErrorPercent;
    double jerkControlMultiplier;

    double p;
    double i;
    double d;

    double position;    //In degrees
    double netOutput;

    public TurretPIDController(Double startPositionTicks, Double target, double p, double i, double d) {
        this.startPosition = (startPositionTicks * 360) / ticksPerRev;
        this.target = target;
        this.p = p;
        this.i = i;
        this.d = d;

        startingError = ensureNonZero(startPosition - target);
        //Must ensure starting errors are not 0 to avoid NaN
        RobotLogger.dd(TAG, "Controller startingError: " + startingError);
    }

    public double update(Double positionTicks, int prevLoopTime) {
        position = (positionTicks * 360) / ticksPerRev;
        positionHistoryLocal.add(position);
        outputs = new ArrayList<>();
        outputs.clear();
//        RobotLogger.dd(TAG, "robotPose: " + robotPose.toString());
        RobotLogger.dd(TAG, "target: " + target);
        currentError = position - target;
        errorHistory.add(currentError);

        //Percent errors always positive! Sign is checked later.
        currentErrorPercent = Math.abs(currentError / startingError);
        errorHistoryPercents.add(currentErrorPercent);

        RobotLogger.dd(TAG, "currentError: " + errorHistory.get(errorHistory.size() - 1));
        RobotLogger.dd(TAG, "percentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1));



        //proportional calculator: outputs present error * p
        double pOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = 0.90 + 4 * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1)));
        if (jerkControlMultiplier > 1) {
            jerkControlMultiplier = 1;
        }
        if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
            pOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1) * p);
        }
        else {
            pOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1) * p);
        }

        //integral calculator: outputs integral of all previous yErrors * yi
        double iOutput = 0.0;
        for (Double errorPercent : errorHistoryPercents) {
            iOutput += errorPercent;
        }
        if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
            iOutput = -iOutput * (i/500);    //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }
        else {
            iOutput = iOutput * (i/500);     //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }


        //derivative calculator: outputs approximate derivative (using difference quotient) * yd
        double dOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dOutput = 0.0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dOutput = -(d * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1) - errorHistoryPercents.get(errorHistoryPercents.size() - 2))) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dOutput = (d * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1) - errorHistoryPercents.get(errorHistoryPercents.size() - 2))) / prevLoopTime;
            }
        }
        RobotLogger.dd(TAG, "PID: " + pOutput + ", " + iOutput + ", " + dOutput);
        netOutput = pOutput+iOutput+dOutput;
        return (pOutput+iOutput+dOutput);
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
