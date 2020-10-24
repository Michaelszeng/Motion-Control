package org.firstinspires.ftc.teamcode.util;

public class PurePursuitPathPoint {
    public double x;
    public double y;
    public double h;
    public boolean isVertex;

    public PurePursuitPathPoint(double x, double y, double h, boolean isVertex) {
        this.x = x;
        this.y = y;
        this.h = h;
        this.isVertex = isVertex;
    }
}
