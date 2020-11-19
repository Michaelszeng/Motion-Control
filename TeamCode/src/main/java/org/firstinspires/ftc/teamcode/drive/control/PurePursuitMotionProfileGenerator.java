package org.firstinspires.ftc.teamcode.drive.control;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.util.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.pj2.Cubic;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxJ;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngA;

public class PurePursuitMotionProfileGenerator {
    final String TAG = "PurePursuitMotionProfileGenerator";

    PurePursuitPath path;
    Cubic cubic = new Cubic();
    double duration = 0.0;

    public PurePursuitMotionProfileGenerator(PurePursuitPath path) {
        this.path = path;
    }

    public PurePursuitPath generateProfiles() {
        double t_MaxAccelReached = (maxA - 0) / maxJ;
        double s_MaxAccelReached = 0.1667 * maxJ * Math.pow(t_MaxAccelReached, 3);
//        RobotLogger.dd(TAG, "t_MaxAccelReached: " + t_MaxAccelReached);
//        RobotLogger.dd(TAG, "s_MaxAccelReached: " + s_MaxAccelReached);


        int index_s_MaxAccelReached = 0;
        double currentDistance = 0;

        double prevSmallestDiff = 9999.9;
        double smallestDiff = 9998.9;
        double currentDiff;

        PurePursuitPathPoint prevPose = path.path1.get(0);
        int counter = 0;
        while (smallestDiff < prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
            currentDistance += Math.hypot(path.path1.get(counter).x, path.path1.get(counter).y);
//            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
            currentDiff = Math.abs(s_MaxAccelReached - currentDistance);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
            prevSmallestDiff = smallestDiff;
            if (currentDiff < prevSmallestDiff) {
                smallestDiff = currentDiff;
            }
            else {
                break;
            }
            counter++;
        }
        index_s_MaxAccelReached = counter;
//        RobotLogger.dd(TAG, "counter: " + counter);

        double v0 = 0.0;
        double a0 = 0.0;
        double dx;
        double v;
        double a;
        double t;   //Time increment; duration is the total time elasped to the pose
        for (int i=0; i<counter; i++) {     //This loop sets the targetV and targetA for all points between t=0 and the max acceleration pose
            dx = Math.hypot(path.path1.get(i).x, path.path1.get(i).y);
            //Equation: 0 = -dx + v0t + 1/2 at^2 + 1/6 jt^3
            cubic.solve(0.1667 * maxJ, 0.5 * a0, v0, -dx);
            if (cubic.nRoots == 1) {
                t = cubic.x1;
            }
            else {
                t = Math.max(cubic.x1, Math.max(cubic.x2, cubic.x3));   //Only one solution will be positive with the given domains, so I can just find the max of the 3 as t
            }
            duration += t;

            //Equation: v = v0 + a0t + 1/2 jt^2
            v = v0 + a0*t + 0.5*maxJ*Math.pow(t, 2);
            a = a0 + maxJ*t;

            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, a, duration));
            v0 = v;
            a0 = a;
        }


//        double t_MaxVReached = (maxV - v0) / maxA;
//        double s_maxVReached = s_MaxAccelReached + v0*t_MaxVReached + 0.5*maxA*Math.pow(t_MaxVReached, 2);
//
//        int index_s_MaxVReached = index_s_MaxAccelReached;
//        prevPose = path.path1.get(index_s_MaxAccelReached);
//        counter = index_s_MaxAccelReached;
//        while (smallestDiff < prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
//            currentDistance += Math.hypot(path.path1.get(counter).x, path.path1.get(counter).y);
//            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
//            currentDiff = Math.abs(s_MaxAccelReached - currentDistance);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
//            prevSmallestDiff = smallestDiff;
//            if (currentDiff < prevSmallestDiff) {
//                smallestDiff = currentDiff;
//            }
//            else {
//                break;
//            }
//            counter++;
//        }
//        index_s_MaxAccelReached = counter;
//        RobotLogger.dd(TAG, "counter: " + counter);

        return null;
    }

    public void exportProfiles() {
        //Parallel Strings
        String times = "";
        String velocities = "";
        String accelerations = "";
        for (PurePursuitPathPoint p : path.path1) {
            if (String.valueOf(p.t) == "NaN") {

            }
            else {
                RobotLogger.dd(TAG, "Appending: " + String.valueOf(p.t) + ", " + String.valueOf(p.velocity)+ ", " + String.valueOf(p.acceleration));
                times = times + p.t;
                times = times + ",";
                velocities = velocities + p.velocity;
                velocities = velocities + ",";
                accelerations = accelerations + p.acceleration;
                accelerations = accelerations + ",";
            }
        }
        RobotLogger.dd(TAG, "times: " + times);
        RobotLogger.dd(TAG, "velocities: " + velocities);
        RobotLogger.dd(TAG, "accelerations: " + accelerations);
    }

}
