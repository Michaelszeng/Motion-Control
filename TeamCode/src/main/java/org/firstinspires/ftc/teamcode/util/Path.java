package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class Path {
    public ArrayList<PurePursuitPath> PPPaths = new ArrayList<>();
    public int currentPath;

    public Path() {
        currentPath = 0;
    }

    public Path(PurePursuitPath p1) {
        currentPath = 0;
        PPPaths.add(p1);
    }

    public Path(PurePursuitPath p1, PurePursuitPath p2) {
        currentPath = 0;
        PPPaths.add(p1);
        PPPaths.add(p2);
    }



    public void append(PurePursuitPath PPPath2) throws Exception {
        if (PPPaths.size() == 0) {  //Avoid out of bounds error
            PPPaths.add(PPPath2);
        }
        else {
            ArrayList<PurePursuitPathPoint> path1 = PPPaths.get(PPPaths.size() - 1).path1;
            ArrayList<PurePursuitPathPoint> path2 = PPPath2.path1;

            //Check if the end point and start points match
            if (path1.get(path1.size() - 1).x == path2.get(0).x && path1.get(path1.size() - 1).y == path2.get(0).y && path1.get(path1.size() - 1).h == path2.get(0).h && path1.get(path1.size() - 1).isVertex == path2.get(0).isVertex) {
                PPPaths.add(PPPath2);
            }
            else {
                throw new Exception("Cannot Join Paths. End and Start PurePursuitPathPoint are not the same.");
            }
        }
    }

    public void incrementPath() {
        currentPath = currentPath + 1;
    }

    public PurePursuitPath get(int i) {
        return PPPaths.get(i);
    }
}
