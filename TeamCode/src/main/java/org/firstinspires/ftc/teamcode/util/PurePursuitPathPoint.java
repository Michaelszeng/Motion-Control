package org.firstinspires.ftc.teamcode.util;

public class PurePursuitPathPoint {
    /*
    IMPORTANT: velocity and acceleration are always positive; the sign is adjusted in the trig functions in PIDController.
    angVelocity and angAcceleration have correct signs already.
     */


    public double x;
    public double y;
    public double h;
    public boolean isVertex;
    public double velocity;
    public double acceleration;
    public double t;
    public double angVelocity;
    public double angAcceleration;

    @Override
    public String toString() {
        return "{" +
                "x=" + String.format("%.4f", x) +
                ", y=" + String.format("%.4f", y) +
                ", h=" + String.format("%.4f", h) +
                ", isVertex=" + isVertex +
                ", velocity=" + String.format("%.4f", velocity) +
                ", acceleration=" + String.format("%.4f", acceleration) +
                ", t=" + String.format("%.4f", t) +
                ", angVelocity=" + String.format("%.4f", angVelocity) +
                ", angAcceleration=" + String.format("%.4f", angAcceleration) +
                '}';
    }

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        velocity = 0.0;
        acceleration = 0.0;
        t = 0.0;
        angVelocity = 0.0;
        angAcceleration = 0.0;
    }

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex, double velocity, double acceleration, double t) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.t = t;
        angVelocity = 0.0;
        angAcceleration = 0.0;
    }

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex, double velocity, double acceleration, double t, double angVelocity, double angAcceleration) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.t = t;
        this.angVelocity = angVelocity;
        this.angAcceleration = angAcceleration;
    }

    public void set(double x, double y, double h, boolean isVertex, double velocity, double acceleration, double t) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.t = t;
        angVelocity = 0.0;
        angAcceleration = 0.0;
    }

    public void set(double x, double y, double h, boolean isVertex, double velocity, double acceleration, double t, double hVelocity, double hAcceleration) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.t = t;
        this.angVelocity = hVelocity;
        this.angAcceleration = hAcceleration;
    }
}
