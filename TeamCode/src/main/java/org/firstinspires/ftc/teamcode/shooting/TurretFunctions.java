package org.firstinspires.ftc.teamcode.shooting;

import java.util.ArrayList;

public class TurretFunctions {
    public String TAG = "TurretFunctions";

    final static double leftBound = 0;
    final static double rightBound = 480;

    final double shooterY = 13.5;
    final double legalY = 5.0 * 12 + shooterY;
    final double legalX = 16.0 * 12;
    final double g = 9.806;
    double xAcceleration = 0.0;

    final static double threshold = 20.0;    //degrees

    public boolean determineShotLegality(double velocity, double angle) {
        double v0y = velocity * Math.sin(angle);
        double v0x = velocity * Math.cos(angle);
        double t = v0y / g;
        double dx = v0x * t;
        double dy = v0y * t - 0.5 * g * Math.pow(t, 2);

        if (2 * dx > legalX || dy > legalY) {
            return false;
        }
        return true;
    }

    public static double normalizeTarget(double currentPosition, double targetH) {
        double newTarget;

        double shortestDisplacementToTarget = normalizeAngle(targetH - currentPosition);
        double targetPosition = shortestDisplacementToTarget + currentPosition;

        if (targetPosition > rightBound - threshold) {
            newTarget = targetPosition - 360;
            return newTarget;
        }
        else if (targetPosition < leftBound + threshold) {
            newTarget = targetPosition + 360;
            return newTarget;
        }
        return targetH;
    }

    public static double normalizeAngle(double angle) {
        //Ensures angle turn is between 180 and -180
        while(angle < -180) {
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }

    public ArrayList<Double> quadSolver(double a, double b, double c) {
        double discriminant = Math.pow(b, 2) - (4 * a * c);
        try {
            double rootDiscriminant = Math.sqrt(discriminant);
            double solution1 = (-b + rootDiscriminant) / (2 * a);
            double solution2 = (-b - rootDiscriminant) / (2 * a);
            ArrayList<Double> solutions = new ArrayList<>();
            solutions.add(solution1);
            solutions.add(solution2);
            return solutions;
        }
        catch (Exception e) {
            return null;
        }
    }
}
