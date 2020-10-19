package org.firstinspires.ftc.teamcode.shooting;

import java.util.ArrayList;

public class TurretFunctions {
    public String TAG = "TurretFunctions";
    double shooterY = 13.5;
    double legalY = 5.0 * 12 + shooterY;
    double legalX = 16.0 * 12;
    double g = 9.806;
    double xAcceleration = 0.0;

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
