package org.firstinspires.ftc.teamcode.util;

public class Angle {
    public static double norm(double angle) {
        double tau = 2 * Math.PI;
        double r = angle % tau;
        r = (r + tau) % tau;
        return r;
    }
    public static boolean epsilonEquals(double a, double b) {
        double EPSILON = 1e-6;
        boolean r = ((a - b) < EPSILON) ? true : false;
        return r;
    }

}
