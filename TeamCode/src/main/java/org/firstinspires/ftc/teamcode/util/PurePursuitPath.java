package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class PurePursuitPath {
    public ArrayList<Pose2d> path;

    public PurePursuitPath(ArrayList<Pose2d> path) {
        this.path = path;
    }

    public int size() {
        return path.size();
    }

}
