package org.firstinspires.ftc.teamcode.drive.control;

import org.firstinspires.ftc.teamcode.util.MathFunctions.Quadratic;
import org.firstinspires.ftc.teamcode.util.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.MathFunctions.Cubic;


import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxJ;

public class PurePursuitMotionProfileGenerator {
    final String TAG = "PurePursuitMotionProfileGenerator";

    PurePursuitPath path;
    Cubic cubic = new Cubic();
    Quadratic quad = new Quadratic();
    double duration = 0.0;

    public PurePursuitMotionProfileGenerator(PurePursuitPath path) {
        this.path = path;
    }

    public PurePursuitPath generateProfiles() {
        //SECTION 1
        double t_MaxAccelReached = (maxA - 0) / maxJ;
        double s_MaxAccelReached = 0.1667 * maxJ * Math.pow(t_MaxAccelReached, 3);
//        RobotLogger.dd(TAG, "t_MaxAccelReached: " + t_MaxAccelReached);
//        RobotLogger.dd(TAG, "s_MaxAccelReached: " + s_MaxAccelReached);

        double currentDistance = 0;

        double prevSmallestDiff = 9999.9;
        double smallestDiff = 9998.9;
        double currentDiff;

        int counter = 0;
        while (smallestDiff < prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
            if (counter == 0) {
                currentDistance += Math.hypot(path.path1.get(counter).x, path.path1.get(counter).y);
            }
            else {
                currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
            }
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
        int index_s_MaxAccelReached = counter;
        RobotLogger.dd(TAG, "Section 1 counter: " + counter);

        double v0 = 0.0;
        double a0 = 0.0;
        double dx;
        double v;
        double a;
        double t=0;   //Time increment; duration is the total time elasped to the pose
        for (int i=0; i<index_s_MaxAccelReached; i++) {     //This loop sets the targetV and targetA for all points between t=0 and the max acceleration pose
            if (i == 0) {
                dx = Math.abs(Math.hypot(path.path1.get(i).x, path.path1.get(i).y));
            }
            else {
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
            }
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
            if (a > maxA) {
                a = maxA;
            }

            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, a, duration));
            v0 = v;
            a0 = a;
        }
        //END SECTION 1



        //SECTION 2
        double t_InflectReached = (maxV - (2*v0)) / maxA;
        double s_InflectReached = s_MaxAccelReached + v0*t_InflectReached + 0.5*maxA*Math.pow(t_InflectReached, 2);

        currentDistance = 0;

        prevSmallestDiff = 9999.9;
        smallestDiff = 9998.9;
        counter = index_s_MaxAccelReached;
        while (smallestDiff < prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max velocity with constant accel should be reached
            currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
            currentDiff = Math.abs(s_InflectReached - currentDistance);
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
        int index_s_InflectReached = counter;
        RobotLogger.dd(TAG, "counter: " + counter);

        for (int i=index_s_MaxAccelReached; i<index_s_InflectReached; i++) {     //This loop sets the targetV and targetA for all points between reaching max accel and reaching max velocity
            dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
//            dx = Math.hypot(path.path1.get(i).x, path.path1.get(i).y);
            //Equation: 0 = -dx + v0t + 1/2 at^2
            quad.solve(0.5 * maxA, v0, -dx);
            if (quad.nRoots == 1) {
                t = quad.x1;
            }
            else if (quad.nRoots == 2) {
                t = Math.max(quad.x1, quad.x2);   //Only one solution will be positive with the given domains, so I can just find the max of the 3 as t
            }
            else {  //Theoretically this should never happen
                t = 0.0;
            }
            duration += t;

            //Equation: v = v0 + a0t
            v = v0 + maxA*t;

            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, maxA, duration));
            v0 = v;
        }
        //END SECTION 2



        //SECTION 3
//        counter = index_s_InflectReached;
//        double dt;
//        double dv;
//        double da;
//        double duration_0AccelReached = duration + t_MaxAccelReached;
//        while (duration < duration_0AccelReached) {
//            dx = Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//            cubic.solve(-0.1667*maxJ, 0.5*a0, v0, -dx);
//            if (cubic.nRoots == 1) {
//                dt = cubic.x1;
//            }
//            else {
//                dt = Math.max(cubic.x1, Math.max(cubic.x2, cubic.x3));   //Only one solution will be positive with the given domains, so I can just find the max of the 3 as t
//            }
//            duration += dt;
//
//            dv = a0*dt - 0.5*maxJ*Math.pow(dt, 2);
//            da = maxJ * dt;
//            v = v0+dv;
//            a = a0+da;
//            path.path1.set(counter, new PurePursuitPathPoint(path.path1.get(counter).x, path.path1.get(counter).y, path.path1.get(counter).h, path.path1.get(counter).isVertex, v, a, duration));
//            v0 = v;
//            a0 = a;
//            counter++;
//        }
//        int index_s_0AccelReached = counter;
//        RobotLogger.dd(TAG, "Section 3 counter: " + counter);



        //Time interval from maxA to 0A
        double t1 = (maxA)/maxJ;
        RobotLogger.dd(TAG, "t1: " + t1);
        double t_0AccelReached = t_MaxAccelReached;  //This is true because the 2 sides of the acceleration trapezoid are the same
        RobotLogger.dd(TAG, "t_0: " + t_0AccelReached);
        //Displacement from maxA to 0A
//        double s_0AccelReached = s_InflectReached + v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) - 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
        double s_0AccelReached = v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) - 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
//        double ds = s_0AccelReached - s_InflectReached;
//        RobotLogger.dd(TAG, "ds: " + ds);
        RobotLogger.dd(TAG, "s_0accelreached: " + s_0AccelReached);

        currentDistance = 0;

        prevSmallestDiff = 9999.9;
        smallestDiff = 9998.9;

        counter = index_s_InflectReached;
        while (smallestDiff < prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
            currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
            currentDiff = Math.abs(s_0AccelReached - currentDistance);
            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
            prevSmallestDiff = smallestDiff;
            if (currentDiff < prevSmallestDiff) {
                smallestDiff = currentDiff;
            }
            else {
                break;
            }
            counter++;
        }
        int index_s_0AccelReached = counter;
        RobotLogger.dd(TAG, "Section 3 counter: " + counter);

        a0 = maxA;
        double prevdt = t;
        double diff1;
        double diff2;
        double diff3;
        double minDiff;
        for (int i=index_s_InflectReached; i<index_s_0AccelReached; i++) {     //This loop sets the targetV and targetA for all points between t=0 and the max acceleration pose
            dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
            //Equation: 0 = -dx + v0t + 1/2 at^2 + 1/6 jt^3
            //t is the time displacement between 2 consecutive Poses
            cubic.solve(-0.1667 * maxJ, 0.5 * a0, v0, -dx);
//            RobotLogger.dd(TAG, "cubic solutions: " + cubic.x1 + ", " + cubic.x2 + ", " + cubic.x3);
            if (cubic.nRoots == 1) {
                t = cubic.x1;
            }
            else {
                diff1 = Math.abs(cubic.x1 - prevdt);
                diff2 = Math.abs(cubic.x2 - prevdt);
                diff3 = Math.abs(cubic.x3 - prevdt);
                minDiff = Math.min(diff1, Math.min(diff2, diff3));   //Only one solution will be positive with the given domains, so I can just find the max of the 3 as t
                if (minDiff == diff1 && cubic.x1 > 0.0) {
//                    RobotLogger.dd(TAG, "t (x1): " + cubic.x1);
                    t = cubic.x1;
                }
                else if (minDiff == diff2  && cubic.x2 > 0.0) {
//                    RobotLogger.dd(TAG, "t (x2): " + cubic.x2);
                    t = cubic.x2;
                }
                else if (minDiff == diff3  && cubic.x3 > 0.0) {
//                    RobotLogger.dd(TAG, "t (x3): " + cubic.x3);
                    t = cubic.x3;
                }
                else {
                    RobotLogger.dd(TAG, "---------------------------------TIME CALCULATION FAILURE---------------------------------");
                }
            }
            prevdt = t;
            duration += t;

            //Equation: v = v0 + a0t + 1/2 jt^2
            v = v0 + a0*t - 0.5*maxJ*Math.pow(t, 2);
            a = a0 - maxJ*t;
            if (v > maxV) {
                v = maxV;
            }
            if (a < 0.0) {
                a = 0.0;
            }

            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, a, duration));
            v0 = v;
            a0 = a;
        }
        //END SECTION 3



        //SECTION 4
//        int index_s_Accelerate = path.path1.size() - index_s_0AccelReached;     //The points that have 0 acceleration are the section between the equal length trapezoids
//        for (int i=index_s_0AccelReached; i<index_s_Accelerate; i++) {
//            dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
////            dx = Math.hypot(path.path1.get(i).x, path.path1.get(i).y);
//            t = dx/maxV;
//            duration += t;
//            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, maxV, 0.0, duration));
//        }
        //END SECTION 4

        return null;
    }

    public void exportProfiles() {
        //Parallel Strings
        String times = "";
        String velocities = "";
        String accelerations = "";
        for (PurePursuitPathPoint p : path.path1) {
            if (p.t == Double.NaN) {

            }
            else {
//                RobotLogger.dd(TAG, "Appending: " + String.valueOf(p.t) + ", " + String.valueOf(p.velocity)+ ", " + String.valueOf(p.acceleration));
                times = times + String.format("%.4f", p.t);
                times = times + ",";
                velocities = velocities + String.format("%.4f", p.velocity);
                velocities = velocities + ",";
                accelerations = accelerations + String.format("%.4f", p.acceleration);
                accelerations = accelerations + ",";
            }
        }
        RobotLogger.dd(TAG, "times: " + times);
        RobotLogger.dd(TAG, "velocities: " + velocities);
        RobotLogger.dd(TAG, "accelerations: " + accelerations);
    }

}
