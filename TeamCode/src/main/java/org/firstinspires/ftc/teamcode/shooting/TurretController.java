package org.firstinspires.ftc.teamcode.shooting;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.teamcode.util.RobotLogger;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.shooting.TurretFunctions;

public class TurretController {
    final String TAG = "TurretController";
    final int ticksPerRev = 767;
    //Set the bounds of the turret rotation so wiring doesn't break


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

    public TurretController(Double startPositionTicks, Double rawTarget, double p, double i, double d) {
        this.startPosition = (startPositionTicks * 360) / ticksPerRev;
        this.p = p;
        this.i = i;
        this.d = d;
        this.target = TurretFunctions.normalizeTarget(startPositionTicks, rawTarget);
        startingError = ensureNonZero(startPosition - target);
        //Must ensure starting errors are not 0 to avoid NaN
        RobotLogger.dd(TAG, "Controller startingError: " + startingError);
    }

    public double update(int positionTicks, int prevLoopTime, double rawTarget) {
        position = (positionTicks * 360) / ticksPerRev;
        positionHistoryLocal.add(position);

        this.target = TurretFunctions.normalizeTarget(positionTicks, rawTarget);

        outputs = new ArrayList<>();
        outputs.clear();
        RobotLogger.dd(TAG, "target: " + target);
        currentError = target - position;
        errorHistory.add(currentError);

        //Percent errors always positive! Sign is checked later.
        currentErrorPercent = Math.abs(currentError / startingError);
        errorHistoryPercents.add(currentErrorPercent);

        RobotLogger.dd(TAG, "currentError: " + errorHistory.get(errorHistory.size() - 1));
        RobotLogger.dd(TAG, "percentError: " + errorHistoryPercents.get(errorHistoryPercents.size() - 1));



        //proportional calculator: outputs present error * p
        double pOutput;
        //added constant = initial speed; multiplier constant = how fast it accelerates (higher = faster)
        jerkControlMultiplier = 0.80 + 4 * (1 - (errorHistoryPercents.get(errorHistoryPercents.size() - 1)));
        if (jerkControlMultiplier > 1) {
            jerkControlMultiplier = 1;
        }
        if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
            pOutput = jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1) * p);
        }
        else {
            pOutput = -jerkControlMultiplier * (errorHistoryPercents.get(errorHistoryPercents.size() - 1) * p);
        }

        //integral calculator: outputs integral of all previous yErrors * yi
        double iOutput = 0.0;
        for (Double errorPercent : errorHistoryPercents) {
            iOutput += errorPercent;
        }
        if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
            iOutput = iOutput * (i/500);    //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }
        else {
            iOutput = -iOutput * (i/500);     //Dividing by large number so the inputted yI value can be a comprehensibly large number
        }


        //derivative calculator: outputs approximate derivative (using difference quotient) * yd
        double dOutput;
        if (errorHistoryPercents.size() < 2) {    //if there only 1 data point, set derivative output to 0
            dOutput = 0.0;
        }
        else {      //if there are 2 or more data points, calculate a derivative over the 1st and 2nd
            if (errorHistory.get(errorHistory.size() - 1) >= 0.0) {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dOutput = (d * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1) - errorHistoryPercents.get(errorHistoryPercents.size() - 2))) / prevLoopTime;
            }
            else {
                //Multiplying by a constant so the inputted yD value can be around 1.0
                dOutput = -(d * 50) * ((errorHistoryPercents.get(errorHistoryPercents.size() - 1) - errorHistoryPercents.get(errorHistoryPercents.size() - 2))) / prevLoopTime;
            }
        }
        RobotLogger.dd(TAG, "PID: " + pOutput + ", " + iOutput + ", " + dOutput);
        netOutput = pOutput+iOutput+dOutput;

        toString(positionTicks, rawTarget, target, netOutput);
        return (pOutput+iOutput+dOutput);
    }

    public void toString(int positionTicks, double rawTarget, double normalizedTarget, double power) {
        RobotLogger.dd(TAG, "positionTicks: " + positionTicks);
        RobotLogger.dd(TAG, "rawTarget: " + rawTarget);
        RobotLogger.dd(TAG, "normalizedTarget: " + normalizedTarget);
        RobotLogger.dd(TAG, "outputPower: " + power);
        RobotLogger.dd(TAG, " ");
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
