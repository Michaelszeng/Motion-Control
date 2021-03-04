package org.firstinspires.ftc.teamcode.drive.control;

import org.firstinspires.ftc.teamcode.util.MathFunctions.Quadratic;
import org.firstinspires.ftc.teamcode.util.PurePursuitMathFunctions;
import org.firstinspires.ftc.teamcode.util.PurePursuitPath;
import org.firstinspires.ftc.teamcode.util.PurePursuitPathPoint;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.MathFunctions.Cubic;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.hD;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxAngV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxV;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.maxJ;

//NOTE: HEADING IS IN RADIANS
public class PurePursuitMotionProfileGenerator {
    final String TAG = "PurePursuitMotionProfileGenerator";

    PurePursuitPath path;
    Cubic cubic = new Cubic();
    Quadratic quad = new Quadratic();
    double duration = 0.0;

    boolean pathTooShortXY = false;
    boolean pathTooShortH = false;
    int pathLength;

    public PurePursuitMotionProfileGenerator(PurePursuitPath path) {
        this.path = path;
        pathLength = path.path1.size()-1;
    }

    public PurePursuitPath generateProfiles() {
        if (path.path1.get(0).x == path.path1.get(pathLength).x && path.path1.get(0).y == path.path1.get(pathLength).y) {

        }

        //XY
        //SECTION 1
        double t_MaxAccelReached = (maxA - 0) / maxJ;
        double s_MaxAccelReached = 0.1667 * maxJ * Math.pow(t_MaxAccelReached, 3);
//        RobotLogger.dd(TAG, "t_MaxAccelReached: " + t_MaxAccelReached);
//        RobotLogger.dd(TAG, "s_MaxAccelReached: " + s_MaxAccelReached);

        double currentDistance = 0;

        double prevSmallestDiff = 9999.9;
        double prevPrevSmallestDiff;
        double smallestDiff = 9998.9;
        double currentDiff;

        int counter = 0;
        while (smallestDiff <= prevSmallestDiff && counter < path.path1.size()) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
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
            if (currentDiff <= prevSmallestDiff) {
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
//                dx = Math.abs(Math.hypot(path.path1.get(i).x, path.path1.get(i).y));  //Issue: this code only works when the path starts at (0,0)
                dx = 0.0;
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
        RobotLogger.dd(TAG, "Section 1 Duration: " + path.path1.get(index_s_MaxAccelReached-1).t);
        //END SECTION 1



        //SECTION 2
        double t_InflectReached = (maxV - (2*v0)) / maxA;
//        double s_InflectReached = s_MaxAccelReached + v0*t_InflectReached + 0.5*maxA*Math.pow(t_InflectReached, 2);
        double s_InflectReached = v0*t_InflectReached + 0.5*maxA*Math.pow(t_InflectReached, 2);

        currentDistance = 0;

        prevSmallestDiff = 9999.9;
        smallestDiff = 9998.9;
        counter = index_s_MaxAccelReached;
        while (smallestDiff <= prevSmallestDiff && counter < pathLength-1) { //This loop finds the index of the PurePursuitPathPose where max velocity with constant accel should be reached
            currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
            currentDiff = Math.abs(s_InflectReached - currentDistance);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
            prevSmallestDiff = smallestDiff;
            if (currentDiff <= prevSmallestDiff) {
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
        RobotLogger.dd(TAG, "Section 2 Duration: " + path.path1.get(index_s_InflectReached-1).t);
        //END SECTION 2



        //SECTION 3
        //Time interval from maxA to 0A
        double t_0AccelReached = t_MaxAccelReached;  //This is true because the 2 sides of the acceleration trapezoid are the same
        RobotLogger.dd(TAG, "t_0: " + t_0AccelReached);
        //Displacement from maxA to 0A
//        double s_0AccelReached = s_InflectReached + v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) - 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
        double s_0AccelReached = v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) - 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
        //        RobotLogger.dd(TAG, "s_0accelreached: " + s_0AccelReached);
//        double test1 = s_InflectReached + v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) - 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
//        double test2 = s_InflectReached + v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) + 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
//        double test3 = v0*t_0AccelReached + 0.5*maxA*Math.pow(t_0AccelReached, 2) + 0.1667*maxJ*Math.pow(t_0AccelReached, 3);
//        RobotLogger.dd(TAG, "s0Accel1: " + test1);
//        RobotLogger.dd(TAG, "s0Accel2: " + test2);
//        RobotLogger.dd(TAG, "s0Accel3: " + test3);
//        double ds = s_0AccelReached - s_InflectReached;
//        RobotLogger.dd(TAG, "ds: " + ds);

        currentDistance = 0;
        prevSmallestDiff = 9999.9;
        prevPrevSmallestDiff = 9999.9;
        smallestDiff = 9998.9;

        counter = index_s_InflectReached;
        while ((smallestDiff <= prevSmallestDiff|| smallestDiff <= prevPrevSmallestDiff) && counter < pathLength-1) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
            currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
            currentDiff = Math.abs(s_0AccelReached - currentDistance);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
            prevSmallestDiff = smallestDiff;
            if (currentDiff <= prevSmallestDiff) {
                smallestDiff = currentDiff;
            }
            else {
                break;
            }
            prevPrevSmallestDiff = prevSmallestDiff;
            counter++;
        }
        int index_s_0AccelReached = counter;
        if (index_s_0AccelReached > (int) pathLength/2) {
            pathTooShortXY = true;
        }
        RobotLogger.dd(TAG, "Section 3 counter: " + counter);

        if (!pathTooShortXY) {
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
                else {  //There are 3 solutiosn to the cubic; I find the correct one below
                    diff1 = Math.abs(cubic.x1 - prevdt);
                    diff2 = Math.abs(cubic.x2 - prevdt);
                    diff3 = Math.abs(cubic.x3 - prevdt);
                    minDiff = Math.min(diff1, Math.min(diff2, diff3));
                    if (minDiff == diff1 && cubic.x1 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x1): " + cubic.x1);
                        t = cubic.x1;
                    }
                    else if (minDiff == diff2  && cubic.x2 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x2): " + cubic.x2);
                        t = cubic.x2;
                    }
                    else if (minDiff == diff3  && cubic.x3 >= 0.0) {
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
            RobotLogger.dd(TAG, "Section 3 Duration: " + path.path1.get(index_s_0AccelReached-1).t);
            //END SECTION 3



            //SECTION 4
            int index_s_Accelerate = pathLength - index_s_0AccelReached;     //The points that have 0 acceleration are the section between the equal length trapezoids
            for (int i=index_s_0AccelReached; i<index_s_Accelerate; i++) {
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
    //            dx = Math.hypot(path.path1.get(i).x, path.path1.get(i).y);
                t = dx/maxV;
                duration += t;
                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, maxV, 0.0, duration));
            }
            RobotLogger.dd(TAG, "Section 4 Duration: " + path.path1.get(index_s_Accelerate-1).t);
            //END SECTION 4



            //SECTION 5
            //Time interval from 0A to -maxA
            double t_MaxAccelReached2 = t_0AccelReached;  //This is true because the 2 sides of the acceleration trapezoid are the same and the 2 acceleration trapezoids are the same
            //Displacement from 0A to -maxA
            double s_MaxAccelReached2 = v0*t_MaxAccelReached2 - 0.1667*maxJ*Math.pow(t_MaxAccelReached2, 3);    //a0 = 0
            RobotLogger.dd(TAG, "s_MaxAccelreached2: " + s_MaxAccelReached2);

            currentDistance = 0;
            prevSmallestDiff = 9999.9;
            prevPrevSmallestDiff = 9999.9;
            smallestDiff = 9998.9;

            counter = index_s_Accelerate;
            while ((smallestDiff <= prevSmallestDiff || smallestDiff <= prevPrevSmallestDiff) && counter < pathLength-1) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
                currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//                RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
                currentDiff = Math.abs(s_MaxAccelReached2 - currentDistance);
//                RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//                RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
                prevSmallestDiff = smallestDiff;
                if (currentDiff <= prevSmallestDiff) {
                    smallestDiff = currentDiff;
                }
                else {
                    break;
                }
                prevPrevSmallestDiff = prevSmallestDiff;
                counter++;
            }
            int index_s_MaxAccelReached2 = counter;
            RobotLogger.dd(TAG, "Section 5 counter: " + counter);

            a0 = 0;
            prevdt = t;
            for (int i=index_s_Accelerate; i<index_s_MaxAccelReached2; i++) {     //This loop sets the targetV and targetA for all points between t=0 and the max acceleration pose
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
                //Equation: 0 = -dx + v0t + 1/2 at^2 + 1/6 jt^3
                //t is the time displacement between 2 consecutive Poses
                cubic.solve(-0.1667 * maxJ, 0.5 * a0, v0, -dx);
    //            RobotLogger.dd(TAG, "cubic solutions: " + cubic.x1 + ", " + cubic.x2 + ", " + cubic.x3);
                if (cubic.nRoots == 1) {
                    t = cubic.x1;
                }
                else {  //There are 3 solutiosn to the cubic; I find the correct one below
                    diff1 = Math.abs(cubic.x1 - prevdt);
                    diff2 = Math.abs(cubic.x2 - prevdt);
                    diff3 = Math.abs(cubic.x3 - prevdt);
                    minDiff = Math.min(diff1, Math.min(diff2, diff3));
                    if (minDiff == diff1 && cubic.x1 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x1): " + cubic.x1);
                        t = cubic.x1;
                    }
                    else if (minDiff == diff2  && cubic.x2 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x2): " + cubic.x2);
                        t = cubic.x2;
                    }
                    else if (minDiff == diff3  && cubic.x3 >= 0.0) {
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
                if (v < 0.0) {
                    v = 0.0;
                }
                if (a > maxA) {
                    a = maxA;
                }

                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, a, duration));
                v0 = v;
                a0 = a;
            }
            RobotLogger.dd(TAG, "Section 5 Duration: " + path.path1.get(index_s_MaxAccelReached2-1).t);
            //END SECTION 5



            //SECTION 6
            //Time displacement between -maxA and 0A
            double t_InflectReached2 = t_InflectReached;    //two acceleration trapezoids are the same
            //Displacement between -maxA and 0A
            double s_InflectReached2 = v0*t_InflectReached2 - 0.5*maxA*Math.pow(t_InflectReached2, 2);

            currentDistance = 0;

            prevSmallestDiff = 9999.9;
            smallestDiff = 9998.9;
            counter = index_s_MaxAccelReached2;
            while (smallestDiff <= prevSmallestDiff && counter < pathLength-1) { //This loop finds the index of the PurePursuitPathPose where max velocity with constant accel should be reached
                currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
    //            RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
                currentDiff = Math.abs(s_InflectReached2 - currentDistance);
    //            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
    //            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
                prevSmallestDiff = smallestDiff;
                if (currentDiff <= prevSmallestDiff) {
                    smallestDiff = currentDiff;
                }
                else {
                    break;
                }
                counter++;
            }
            int index_s_InflectReached2 = counter;
            RobotLogger.dd(TAG, "counter: " + counter);

            prevdt = t;
            for (int i=index_s_MaxAccelReached2; i<index_s_InflectReached2; i++) {     //This loop sets the targetV and targetA for all points between reaching max accel and reaching max velocity
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
    //            dx = Math.hypot(path.path1.get(i).x, path.path1.get(i).y);
                //Equation: 0 = -dx + v0t + 1/2 at^2
                quad.solve(-0.5 * maxA, v0, -dx);
                if (quad.nRoots == 1) {
                    t = quad.x1;
                }
                else if (quad.nRoots == 2) {
                    diff1 = Math.abs(quad.x1 - prevdt);
                    diff2 = Math.abs(quad.x2 - prevdt);
                    minDiff = Math.min(diff1, diff2);
                    if (minDiff == diff1 && quad.x1 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x1): " + cubic.x1);
                        t = quad.x1;
                    }
                    else if (minDiff == diff2  && quad.x2 >= 0.0) {
    //                    RobotLogger.dd(TAG, "t (x2): " + cubic.x2);
                        t = quad.x2;
                    }
                    else {  //Theoretically this should never happen
                        t = 0.0;
                    }
                }
                else {  //Theoretically this should never happen
                    t = 0.0;
                }
                prevdt = t;
                duration += t;

                //Equation: v = v0 + a0t
                v = v0 - maxA*t;
                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, -maxA, duration));
                v0 = v;
            }
            RobotLogger.dd(TAG, "Section 6 Duration: " + path.path1.get(index_s_InflectReached2-1).t);
            //END SECTION 6



            //SECTION 7
            //Time interval from -maxA to 0A
            double t_End = t_MaxAccelReached2;  //This is true because the 2 sides of the acceleration trapezoid are the same and the 2 acceleration trapezoids are the same
            //Displacement from 0A to -maxA
            double s_End = v0*t_End - 0.5*maxA*Math.pow(t_End, 2) + 0.1667*maxJ*Math.pow(t_End, 3);
            RobotLogger.dd(TAG, "s_End:" + s_End);

            currentDistance = 0;
            prevSmallestDiff = 9999.9;
            prevPrevSmallestDiff = 9999.9;
            smallestDiff = 9998.9;

            counter = index_s_InflectReached2;
            while ((smallestDiff <= prevSmallestDiff || smallestDiff <= prevPrevSmallestDiff) && counter < pathLength-1) { //This loop finds the index of the PurePursuitPathPose where max acceleration should be reached
                currentDistance += Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y);
//                RobotLogger.dd(TAG, "currentDistance: " + currentDistance);
                currentDiff = Math.abs(s_End - currentDistance);
//                RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//                RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
                prevSmallestDiff = smallestDiff;
                if (currentDiff <= prevSmallestDiff) {
                    smallestDiff = currentDiff;
                }
                else {
                    break;
                }
                prevPrevSmallestDiff = prevSmallestDiff;
                counter++;
            }
//            int index_s_End = counter;
//            RobotLogger.dd(TAG, "Section 7 counter: " + counter);

            prevdt = t;
            for (int i=index_s_InflectReached2; i<=pathLength; i++) {     //This loop sets the targetV and targetA for all points between t=0 and the max acceleration pose
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
                //Equation: 0 = -dx + v0t + 1/2 at^2 + 1/6 jt^3
                //t is the time displacement between 2 consecutive Poses
                cubic.solve(0.1667 * maxJ, 0.5 * a0, v0, -dx);
    //            RobotLogger.dd(TAG, "cubic solutions: " + cubic.x1 + ", " + cubic.x2 + ", " + cubic.x3);
                if (cubic.nRoots == 1) {
                    t = cubic.x1;
                }
                else {  //There are 3 solutions to the cubic; I find the correct one below
                    diff1 = Math.abs(cubic.x1 - prevdt);
                    diff2 = Math.abs(cubic.x2 - prevdt);
                    diff3 = Math.abs(cubic.x3 - prevdt);
                    minDiff = Math.min(diff1, Math.min(diff2, diff3));
                    if (minDiff == diff1 && cubic.x1 >= 0.0 && cubic.x1 < 999999) {
    //                    RobotLogger.dd(TAG, "t (x1): " + cubic.x1);
                        t = cubic.x1;
                    }
                    else if (minDiff == diff2  && cubic.x2 >= 0.0 && cubic.x2 < 999999) {
    //                    RobotLogger.dd(TAG, "t (x2): " + cubic.x2);
                        t = cubic.x2;
                    }
                    else if (minDiff == diff3  && cubic.x3 >= 0.0 && cubic.x3 < 999999) {
    //                    RobotLogger.dd(TAG, "t (x3): " + cubic.x3);
                        t = cubic.x3;
                    }
                    else {
                        RobotLogger.dd(TAG, "---------------------------------TIME CALCULATION FAILURE---------------------------------");
                    }
                }


                if (i >= pathLength) {   //Set the last coordinate to 0 targetV and targetA
                    v = 0.0;
                    a = 0.0;
                    duration = path.path1.get(i-1).t;
//                    RobotLogger.dd(TAG, "end of path duration: " + duration);
//                    RobotLogger.dd(TAG, "end of path t: " + t);
                }
                else {
                    prevdt = t;
                    duration += t;
//                    RobotLogger.dd(TAG, "t: " + t);
//                    RobotLogger.dd(TAG, "v0: " + v0);
//                    RobotLogger.dd(TAG, "a0: " + a0);
                    //Equation: v = v0 + a0t + 1/2 jt^2
                    v = v0 + a0*t + 0.5*maxJ*Math.pow(t, 2);
                    a = a0 + maxJ*t;
                    if (v < 0.0) {
                        v = 0.0;
                    }
                    if (a > 0.0) {
                        a = 0.0;
                    }
                }

                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, a, duration));
                v0 = v;
                a0 = a;
            }
            RobotLogger.dd(TAG, "Section 7 Duration: " + path.path1.get(pathLength).t);
            //END SECTION 7
        }
        else {  //If the path is too short
            //Path Motion Profile will be reduced to instantaneous-jerk profile for simplicity
            RobotLogger.dd(TAG, "CREATING INSTANTANEOUS-JERK MOTION PROFILE FOR SHORT PATH");
            duration = 0.0;
            //SECTION 2
            counter = 0;
            a = maxA;
            v0 = 0.0;
            v = 0.0001;
            while (counter <= (int) pathLength/2 || v0 > maxV) {
                if (counter == 0) {
                    dx = Math.abs(Math.hypot(path.path1.get(counter).x, path.path1.get(counter).y));
                }
                else {
                    dx = Math.abs(Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y));
                }
                v = Math.sqrt(Math.pow(v0, 2) + 2*a*dx);
                t = (v-v0)/a;
                if (v > maxV) {
                    v = maxV;
                }
                duration += t;
                path.path1.set(counter, new PurePursuitPathPoint(path.path1.get(counter).x, path.path1.get(counter).y, path.path1.get(counter).h, path.path1.get(counter).isVertex, v, maxA, duration));
                v0 = v;
                counter++;
            }
            index_s_0AccelReached = counter;
            RobotLogger.dd(TAG, "short path index_s_0AccelReached: " + index_s_0AccelReached);
            //END SECTION 2

            //SECTION 4
            for (int i=index_s_0AccelReached; i<=pathLength - index_s_0AccelReached; i++) {
                dx = Math.abs(Math.hypot(path.path1.get(i).x - path.path1.get(i-1).x, path.path1.get(i).y - path.path1.get(i-1).y));
                t = dx/v;
                duration += t;
                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h, path.path1.get(i).isVertex, v, 0.0, duration));
            }
            double index_s_Accel = pathLength - index_s_0AccelReached;
            RobotLogger.dd(TAG, "short path index_s_Accel: " + index_s_Accel);
            //END SECTION 4

            //SECTION 6
            counter = pathLength+2-index_s_0AccelReached;
            a = -maxA;
            v0 = v;
            while (counter <= pathLength) {
                dx = Math.abs(Math.hypot(path.path1.get(counter).x - path.path1.get(counter-1).x, path.path1.get(counter).y - path.path1.get(counter-1).y));
//                RobotLogger.dd(TAG, "dx: " + dx);
                if (Math.pow(v0, 2) + 2*a*dx >= 0) {    //Negative Root
                    v = Math.sqrt(Math.pow(v0, 2) + 2*a*dx);
                }
                else {
                    v = 0.0;
                }
//                RobotLogger.dd(TAG, "v: " + v);
                if (a != 0) {
                    t = (v-v0)/a;
                }
//                RobotLogger.dd(TAG, "t: " + t);
                if (counter == pathLength) {
                    v = 0.0;
                    a = 0.0;
                }
                if (v < 0.0) {
                    v = 0.0;
                }
                duration += t;
                path.path1.set(counter, new PurePursuitPathPoint(path.path1.get(counter).x, path.path1.get(counter).y, path.path1.get(counter).h, path.path1.get(counter).isVertex, v, a, duration));
                v0 = v;
                counter++;
            }
            //END SECTION 6
        }
//        int i = 0;
//        for (PurePursuitPathPoint p : path.path1) {
//            if (p.t == 0.0 && i != 0) {
//                p.t = path.path1.get(i-1).t;
//            }
//        }



        //HEADING
        RobotLogger.dd(TAG, "*******************BEGINNING HEADING CALCULATIONS*******************");
        //ALL CALCULATIONS MUST BE IN RADIANS!!!!!
        double h0 = path.path1.get(0).h;
        double hF = path.path1.get(pathLength).h;
        double hDisplacement = EnsureNonzero(PurePursuitMathFunctions.AngleWrap(hF - h0));
        RobotLogger.dd(TAG, "hDisplacement: " + hDisplacement);

        //SECTION 1
        double t_0AngAReached;
        if (Math.pow(path.path1.get(pathLength).t, 2) < ((4 * hDisplacement)/(maxAngA*Math.signum(hDisplacement)))) {
            pathTooShortH = true;
            RobotLogger.dd(TAG, "------------------------------------------PATH IS TOO SHORT FOR GIVEN HEADING CHANGE, RECALCUATING------------------------------------------");
            PurePursuitPathPoint lastP = path.path1.get(pathLength);
            double t_solvable = (4 * hDisplacement)/(maxAngA*Math.signum(hDisplacement));
            RobotLogger.dd(TAG, "t_solvable: " + t_solvable);
            path.path1.add(new PurePursuitPathPoint(lastP.x, lastP.y, hF, true, lastP.velocity, lastP.acceleration, t_solvable));
            pathLength += 1;
        }
//        quad.solve(-maxAngA, maxAngA*path.path1.get(path.path1.size()-1).t, -hDisplacement);      //Before 12/4
//        quad.solve(-maxAngA, -maxAngA*path.path1.get(path.path1.size()-1).t, -hDisplacement);     //12/4 Afternoon
        quad.solve(-maxAngA*Math.signum(hDisplacement), maxAngA*Math.signum(hDisplacement) * path.path1.get(path.path1.size()-1).t, -hDisplacement);   //12/4 Evening
        RobotLogger.dd(TAG, "a: " + (-maxAngA*Math.signum(hDisplacement)));
        RobotLogger.dd(TAG, "b: " + maxAngA*Math.signum(hDisplacement)*path.path1.get(pathLength).t);
        RobotLogger.dd(TAG, "c: " + (-hDisplacement));
        RobotLogger.dd(TAG, "quad.x1: " + quad.x1);
        RobotLogger.dd(TAG, "quad.x2: " + quad.x2);
        if (quad.nRoots == 1) {
            t_0AngAReached = quad.x1;
        }
        else if (quad.nRoots == 2) {
            if (quad.x1 >= 0.0 && quad.x2 >= 0.0) {
                t_0AngAReached = Math.min(quad.x1, quad.x2);
            }
            else {
                t_0AngAReached = Math.max(quad.x1, quad.x2);
            }
            if (t_0AccelReached < 0.0) {
                RobotLogger.dd(TAG, "*********************************ERROR: NO SOLUTION FOR GIVEN HEADING DISPLACEMENT*********************************");
            }
        }
        else {
            RobotLogger.dd(TAG, "*********************************ERROR: PATH IS TOO SHORT FOR GIVEN HEADING CHANGE*********************************");
            t_0AngAReached = 0.0;
        }

        double h_0AngAReached = 0.5 * maxAngA*Math.signum(hDisplacement) * Math.pow(t_0AngAReached, 2);
        double v_0AngAReached = maxAngA*Math.signum(hDisplacement) * t_0AngAReached;

        if (Math.abs(v_0AngAReached) > maxAngV && pathTooShortH != true) {
            RobotLogger.dd(TAG, "*********************************WARNING: MAX ANGULAR VELOCIY EXCEEDED*********************************");
//            pathTooShortH = true;
//            RobotLogger.dd(TAG, "------------------------------------------PATH IS TOO SHORT FOR GIVEN HEADING CHANGE, RECALCUATING------------------------------------------");
//            PurePursuitPathPoint lastP = path.path1.get(pathLength);
//            double t_solvable = (4 * hDisplacement)/(maxAngA*Math.signum(hF - h0));  //Need a new way to solve for t_solvable
//            RobotLogger.dd(TAG, "t_solvable: " + t_solvable);
//            path.path1.add(new PurePursuitPathPoint(lastP.x, lastP.y, hF, true, lastP.velocity, lastP.acceleration, t_solvable));
//            pathLength += 1;
//
//            quad.solve(-maxAngA*Math.signum(hF-h0), maxAngA*Math.signum(hF-h0) * path.path1.get(pathLength).t, -hDisplacement);
////            RobotLogger.dd(TAG, "a: " + (-maxAngA*Math.signum(hF-h0)));
////            RobotLogger.dd(TAG, "b: " + maxAngA*Math.signum(hF-h0)*path.path1.get(path.path1.size()-1).t);
////            RobotLogger.dd(TAG, "c: " + (-hDisplacement));
////            RobotLogger.dd(TAG, "quad.x1: " + quad.x1);
////            RobotLogger.dd(TAG, "quad.x2: " + quad.x2);
//            if (quad.nRoots == 1) {
//                t_0AngAReached = quad.x1;
//            }
//            else if (quad.nRoots == 2) {
//                if (quad.x1 >= 0.0 && quad.x2 >= 0.0) {
//                    t_0AngAReached = Math.min(quad.x1, quad.x2);
//                }
//                else {
//                    t_0AngAReached = Math.max(quad.x1, quad.x2);
//                }
//                if (t_0AccelReached < 0.0) {
//                    RobotLogger.dd(TAG, "*********************************ERROR: NO SOLUTION FOR GIVEN HEADING DISPLACEMENT*********************************");
//                }
//            }
//            else {
//                RobotLogger.dd(TAG, "*********************************ERROR: PATH IS TOO SHORT FOR GIVEN HEADING CHANGE*********************************");
//                t_0AngAReached = 0.0;
//            }
//            h_0AngAReached = 0.5 * maxAngA*Math.signum(hF - h0) * Math.pow(t_0AngAReached, 2);
//            v_0AngAReached = maxAngA*Math.signum(hF - h0) * t_0AngAReached;
        }
        RobotLogger.dd(TAG, "t_0AngAReached: " + t_0AngAReached);
        RobotLogger.dd(TAG, "h_0AngAReached: " + h_0AngAReached);
        RobotLogger.dd(TAG, "v_0AngAReached: " + v_0AngAReached);

        prevSmallestDiff = 9999.9;
        prevPrevSmallestDiff = 9999.9;
        smallestDiff = 9998.9;

        counter = 0;
        while ((smallestDiff <= prevSmallestDiff || smallestDiff <= prevPrevSmallestDiff) && counter < pathLength/2) { //Exit the loop at reaching halfway of the path, or the final heading will never be reached
            currentDiff = Math.abs(t_0AngAReached - path.path1.get(counter).t);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
            prevSmallestDiff = smallestDiff;
            if (currentDiff <= prevSmallestDiff) {
                smallestDiff = currentDiff;
            }
            else {
                break;
            }
            prevPrevSmallestDiff = prevSmallestDiff;
            counter++;
        }
        int index_0AngAReached = counter;
        RobotLogger.dd(TAG, "index_0AngAReached: " + index_0AngAReached);

        double angV0 = 0;
        double angV;
        double h = h0;
        for (int i=0; i<index_0AngAReached; i++) {
            if (i == 0) {
//                t = path.path1.get(i).t;    //Issue: this only works when the path starts at 0 heading
                t = 0.0;
            }
            else {
                t = path.path1.get(i).t - path.path1.get(i-1).t;
            }
            duration += t;

            //Equation: v = v0 + a0t
            angV = angV0 + maxAngA*Math.signum(hDisplacement) * t;
            if (hDisplacement < 0.0) {  //The limits need to be different based on turning left vs right
                if (angV < v_0AngAReached) {
                    angV = v_0AngAReached;
                }
            }
            else {
                if (angV > v_0AngAReached) {
                    angV = v_0AngAReached;
                }
            }
            //Equation: h = v0t + at^2
            h += angV0*t + 0.5 * maxAngA*Math.signum(hDisplacement) * Math.pow(t, 2);
            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, h, path.path1.get(i).isVertex, path.path1.get(i).velocity, path.path1.get(i).acceleration, path.path1.get(i).t, angV, maxAngA * Math.signum(hDisplacement)));
            angV0 = angV;
        }
        RobotLogger.dd(TAG, "Section 1 Heading Duration: " + path.path1.get(index_0AngAReached-1).t);
        //END SECTION 1

        //SECTION 2
        int index_AngAccelerate;
        if (pathTooShortH) {
            index_AngAccelerate = pathLength;
        }
        else {
            double t_AngAccelerate = path.path1.get(path.path1.size() - 1).t - t_0AngAReached;
            RobotLogger.dd(TAG, "t_0AngAReached: " + t_0AngAReached);
            RobotLogger.dd(TAG, "t_AngAccelerate: " + t_AngAccelerate);
            prevSmallestDiff = 9999.9;
            prevPrevSmallestDiff = 9999.9;
            smallestDiff = 9998.9;

            counter = index_0AngAReached;
            while ((smallestDiff <= prevSmallestDiff || smallestDiff <= prevPrevSmallestDiff) && counter < pathLength-1) { //Exit the loop at reaching halfway of the path, or the final heading will never be reached
                currentDiff = Math.abs(t_AngAccelerate - path.path1.get(counter).t);
//            RobotLogger.dd(TAG, "currentDiff: " + currentDiff);
//            RobotLogger.dd(TAG, "smallestDiff: " + smallestDiff);
                prevSmallestDiff = smallestDiff;
                if (currentDiff <= prevSmallestDiff) {
                    smallestDiff = currentDiff;
                }
                else {
                    counter++;
                    break;
                }
                prevPrevSmallestDiff = prevSmallestDiff;
                counter++;
            }
            index_AngAccelerate = counter;
        }
        v = v_0AngAReached;
        for (int i=index_0AngAReached; i<index_AngAccelerate; i++) {
            t = path.path1.get(i).t - path.path1.get(i-1).t;
            //Equation: h = vt
            h += v * t;
            if (i == pathLength) {
                h = hF;
                v = 0.0;    //Not sure if this line should be included
            }
            path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, h, path.path1.get(i).isVertex, path.path1.get(i).velocity, path.path1.get(i).acceleration, path.path1.get(i).t, v, 0.0));
        }
        RobotLogger.dd(TAG, "Section 2 Heading Duration: " + path.path1.get(index_AngAccelerate-1).t);
        //END SECTION 2

        //SECTION 3
        if (!pathTooShortH) {   //Section 3 doesn't happen if the path is too short
            angV0 = v_0AngAReached;
            a = -maxAngA*Math.signum(hDisplacement);
            for (int i=index_AngAccelerate; i<=pathLength; i++) {
                t = path.path1.get(i).t - path.path1.get(i-1).t;
                //Equation: v = v0 + a0t
                angV = angV0 - maxAngA*Math.signum(hDisplacement) * t;
                //Equation: h = v0t + at^2
                h += angV0*t - 0.5 * maxAngA*Math.signum(hDisplacement) * Math.pow(t, 2);
                if (hDisplacement < 0.0) {
                    if (angV > 0.0) {
                        angV = 0.0;
                    }
                }
                else {
                    if (angV < 0.0) {
                        angV = 0.0;
                    }
                }
                if (i == pathLength) {
                    angV = 0.0;
                    h = hF;
                    RobotLogger.dd(TAG, "final H: " + h);
                }
                path.path1.set(i, new PurePursuitPathPoint(path.path1.get(i).x, path.path1.get(i).y, h, path.path1.get(i).isVertex, path.path1.get(i).velocity, path.path1.get(i).acceleration, path.path1.get(i).t, angV, a));
                angV0 = angV;
            }
        }
        RobotLogger.dd(TAG, "Section 3 Heading Duration: " + path.path1.get(pathLength).t);
        //END SECTION 3

        return null;
    }

    public void exportProfiles() {
        //Parallel Strings
        String times = "";
        String velocities = "";
        String accelerations = "";
        String h = "";
        String angV = "";
        String angA = "";
        int counter = 0;
        for (PurePursuitPathPoint p : path.path1) {
            if (p.t == Double.NaN) {

            }
            else {
                if (counter % 3 == 0 || counter == pathLength) {    //Need to do this weird thing because there is a limit to the length of a single print in cmd and Logcat
    //                RobotLogger.dd(TAG, "Appending: " + String.valueOf(p.t) + ", " + String.valueOf(p.velocity)+ ", " + String.valueOf(p.acceleration));
                    times = times + String.format("%.4f", p.t);
                    times = times + ",";
                    velocities = velocities + String.format("%.4f", p.velocity);
                    velocities = velocities + ",";
                    accelerations = accelerations + String.format("%.4f", p.acceleration);
                    accelerations = accelerations + ",";
                    h = h + String.format("%.4f", p.h);
                    h = h + ",";
                    angV = angV + String.format("%.4f", p.angVelocity);
                    angV = angV + ",";
                    angA = angA + String.format("%.4f", p.angAcceleration);
                    angA = angA + ",";
                }
            }
            counter++;
        }
        RobotLogger.dd(TAG, "times: " + times);
        RobotLogger.dd(TAG, "velocities: " + velocities);
        RobotLogger.dd(TAG, "accelerations: " + accelerations);
        RobotLogger.dd(TAG, "h: " + h);
        RobotLogger.dd(TAG, "hVelocities: " + angV);
        RobotLogger.dd(TAG, "hAccelerations: " + angA);

    }

    public double EnsureNonzero(double d) {
        if (Math.abs(d) < 0.001) {
            return 0.001;
        }
        return d;
    }

}
