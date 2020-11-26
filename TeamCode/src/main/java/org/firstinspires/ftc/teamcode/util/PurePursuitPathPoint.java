package org.firstinspires.ftc.teamcode.util;

public class PurePursuitPathPoint {
    public double x;
    public double y;
    public double h;
    public boolean isVertex;
    public double velocity;
    public double acceleration;
    public double t;
    public double angVelocity;
    public double angAcceleration;

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        velocity = Double.NaN;
        acceleration = Double.NaN;
        t = Double.NaN;
    }

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex, double velocity, double acceleration, double t) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.t = t;
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
