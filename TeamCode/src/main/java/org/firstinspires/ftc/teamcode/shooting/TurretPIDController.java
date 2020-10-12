package org.firstinspires.ftc.teamcode.shooting;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.util.RobotLogger;
import java.util.ArrayList;

public class TurretPIDController {
    final String TAG = "PIDController";

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

    public TurretPIDController(Double startPosition, Double target, double p, double i, double d) {
        this.startPosition = startPosition;
        this.target = target;
        this.p = p;
        this.i = i;
        this.d = d;

        startingError = ensureNonZero(startPosition - target);
        //Must ensure starting errors are not 0 to avoid NaN
        RobotLogger.dd(TAG, "Controller startingError: " + startingError);
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
