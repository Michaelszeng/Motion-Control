package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.w3c.dom.DOMException;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

public class PurePursuitMathFunctions {
    private static String TAG = "PurePursuitMathFunctions";

    public enum ReachedDestination {
        FALSE,
        CHANGE,
        TRUE
    }

    public static Pose2d getNextTargetV1(Pose2d robotPose, double radius, PurePursuitPath path) {
        double lastDistance = 9999;
        //2 smallest differences, because the two intersects between circle and line may be slightly different magnitudes
        double smallestDifference1 = 9999;
        double smallestDifference2 = 9999;
        for (Pose2d pathPose : path.path) {
            double distance = Math.sqrt(Math.pow(robotPose.getX() - pathPose.getX(), 2) + Math.pow(robotPose.getY() - pathPose.getY(), 2));
            double distanceRadiusDifference = distance - radius;
            if (distanceRadiusDifference < smallestDifference1 || distanceRadiusDifference < smallestDifference2) {
                //ensure that the difference that is replaced is the larger of the two smallestDifferences
                if (smallestDifference1 > smallestDifference2) {
                    if (lastDistance < distanceRadiusDifference) {    //this is the correct intersect
                        return pathPose;
                    }
                    smallestDifference1 = distanceRadiusDifference;
                }
                else {
                    if (lastDistance < distanceRadiusDifference) {    //this is the correct intersect
                        return pathPose;
                    }
                    smallestDifference2 = distanceRadiusDifference;
                }


                smallestDifference1 = distanceRadiusDifference;
            }
        }
        return null;
    }

    public static Pose2d getNextTargetV2(Pose2d robotPose, double radius, PurePursuitPath path) {
        double smallestDifference1 = 9999.0;
        double smallestDifference2 = 9999.0;
        int smallestDifference1Index = 0;
        int smallestDifference2Index = 0;
        for (int i=0; i<path.size(); i++) {
            Pose2d pathPose = path.path.get(i);
            double distance = Math.hypot(robotPose.getX() - pathPose.getX(), robotPose.getY() - pathPose.getY());
//            double distance = Math.sqrt(Math.pow(robotPose.getX() - pathPose.getX(), 2) + Math.pow(robotPose.getY() - pathPose.getY(), 2));
            double distanceRadiusDifference = Math.abs(distance - radius);

            if (smallestDifference1 > smallestDifference2) {
                if (distanceRadiusDifference < smallestDifference1) {
                    smallestDifference1 = distanceRadiusDifference;
                    smallestDifference1Index = i;
                }
            }
            else {  //smallestDifference2 > smallestDifference1
                if (distanceRadiusDifference < smallestDifference2) {
                    smallestDifference2 = distanceRadiusDifference;
                    smallestDifference2Index = i;
                }
            }
        }

        //if (smallestDifference1 > 1 && smallestDifference2 > 1) {}

        RobotLogger.dd(TAG, "smallestDifference1: " + smallestDifference1);
        RobotLogger.dd(TAG, "smallestDifference2: " + smallestDifference2);
        RobotLogger.dd(TAG, "smallestDifference1Index: " + smallestDifference1Index);
        RobotLogger.dd(TAG, "smallestDifference2Index: " + smallestDifference2Index);
        RobotLogger.dd(TAG, "intersect1: " + "(" + path.path.get(smallestDifference1Index).getX() + ", " + path.path.get(smallestDifference1Index).getY() + ", " + path.path.get(smallestDifference1Index).getHeading() + ")");
        RobotLogger.dd(TAG, "intersect2: " + "(" + path.path.get(smallestDifference2Index).getX() + ", " + path.path.get(smallestDifference2Index).getY() + ", " + path.path.get(smallestDifference2Index).getHeading() + ")");
        //Choosing the point that is later in the path
        return path.path.get(Math.max(smallestDifference1Index, smallestDifference2Index));

        /*
        //Choosing the point in which the robot has to turn the least to get to
        double angleLocal1 = Math.atan2(path.path.get(smallestDifference1Index).getY() - robotPose.getY(), path.path.get(smallestDifference1Index).getX() - robotPose.getX());
        double angleLocal2 = Math.atan2(path.path.get(smallestDifference2Index).getY() - robotPose.getY(), path.path.get(smallestDifference2Index).getX() - robotPose.getX());
        double angleDifference1 = Math.abs(AngleWrap(angleLocal1 - robotPose.getHeading()));
        double angleDifference2 = Math.abs(AngleWrap(angleLocal2 - robotPose.getHeading()));

        if (angleDifference1 > angleDifference2) {
            return path.path.get(smallestDifference1Index);
        }
        return path.path.get(smallestDifference2Index);
        */
    }

    public static int getNextTargetV3(Pose2d robotPose, double radius, PurePursuitPath path, int prevTargetIndex) {
        //Only the 12 intersections will be considered (so the robot can pass back and for within the area of a circle 6 times
        ArrayList<Double> distRadiusDiffs = new ArrayList<>(Arrays.asList(999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9));
        ArrayList<Integer> diffIndices = new ArrayList<>(Arrays.asList(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));

        //Setting the bounds for all the points in the path to check (maximum bounds is +/- 50 points of the previous target
        int minBound;
        if (prevTargetIndex - 150 < 0) {
            minBound = 0;
        }
        else {
            minBound = prevTargetIndex - 150;
        }
        int maxBound;
        if (prevTargetIndex + 150 > path.path.size() - 1) {
            maxBound = path.path.size() - 1;
        }
        else {
            maxBound = prevTargetIndex + 150;
        }


        for (int i=minBound; i<maxBound; i++) {
            Pose2d pathPose = path.path.get(i);
            double distance = Math.hypot(robotPose.getX() - pathPose.getX(), robotPose.getY() - pathPose.getY());   //Get distance between robot pose and current path pose
            double distanceRadiusDifference = Math.abs(distance - radius);

            //Comparing how close to an intersection it is, to the 12 current closes intersections
            int index = distRadiusDiffs.size() - 1;
            while (distanceRadiusDifference < distRadiusDiffs.get(index)) {
                index -= 1;
                if (index == 0) {
                    break;
                }
            }
//            RobotLogger.dd(TAG, "index for replacement: " + index);
            if (index < 11) {   //If the intersection is one of the 12 closest
                distRadiusDiffs.add(index, distanceRadiusDifference);
                diffIndices.add(index, i);
                distRadiusDiffs.remove(distRadiusDiffs.size() - 1);
                diffIndices.remove(diffIndices.size() - 1);
            }
        }

        for (int i=0; i<diffIndices.size(); i++) {   //Print intersects
            RobotLogger.dd(TAG, "prevTargetIndex: " + prevTargetIndex + "; index: " + diffIndices.get(i) + "; intersect: " + path.path.get(diffIndices.get(i)));
        }

        int maxIndex = 0;
        for (int index : diffIndices) {
            if (index > maxIndex) {
                maxIndex = index;
            }
        }
        RobotLogger.dd(TAG, "maxIndex (targetPose): "+ maxIndex);

        //If the target pose is a previous pose in the path, we know we are at the end of the path and we no longer change the target Pose
        if (maxIndex < prevTargetIndex) {
            return prevTargetIndex;
        }
        return maxIndex;
    }

    //ACTIVE
    public static int getNextTargetV4(Pose2d robotPose, double radius, PurePursuitPath path, int prevTargetIndex) {
        //Only the 12 intersections will be considered (so the robot can pass back and for within the area of a circle 6 times
        ArrayList<Double> distRadiusDiffs = new ArrayList<>(Arrays.asList(999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9, 999.9));
        ArrayList<Integer> diffIndices = new ArrayList<>(Arrays.asList(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0));

        //Setting the bounds for all the points in the path to check (maximum bounds is +/- 50 points of the previous target
        int minBound;
        if (prevTargetIndex - 150 < 0) {
            minBound = 0;
        }
        else {
            minBound = prevTargetIndex - 150;
        }
        int maxBound;
        if (prevTargetIndex + 150 > path.path1.size() - 1) {
            maxBound = path.path1.size() - 1;
        }
        else {
            maxBound = prevTargetIndex + 150;
        }

        //GET 12 BEST INTERSECTS
        for (int i=minBound; i<maxBound; i++) {
            Pose2d pathPose = new Pose2d(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h);  //path.path.get(i);
            double distance = Math.hypot(robotPose.getX() - pathPose.getX(), robotPose.getY() - pathPose.getY());   //Get distance between robot pose and current path pose
            double distanceRadiusDifference = Math.abs(distance - radius);

            //Comparing how close to an intersection it is, to the 12 current closes intersections
            int index = distRadiusDiffs.size() - 1;
            while (distanceRadiusDifference < distRadiusDiffs.get(index)) {
                index -= 1;
                if (index == 0) {
                    break;
                }
            }
//            RobotLogger.dd(TAG, "index for replacement: " + index);
            if (index < 11) {   //If the intersection is one of the 12 closest
                distRadiusDiffs.add(index, distanceRadiusDifference);
                diffIndices.add(index, i);
                distRadiusDiffs.remove(distRadiusDiffs.size() - 1);
                diffIndices.remove(diffIndices.size() - 1);
            }
        }

        //PRINT INTERSECTS
        for (int i=0; i<diffIndices.size(); i++) {
            RobotLogger.dd(TAG, "prevTargetIndex: " + prevTargetIndex + "; index: " + diffIndices.get(i) + "; intersect: " + new Pose2d(path.path1.get(diffIndices.get(i)).x, path.path1.get(diffIndices.get(i)).y, path.path1.get(diffIndices.get(i)).h));
        }

        //PICK THE INTERSECT WITH THE MAX INDEX
        int maxIndex = 0;
        for (int index : diffIndices) {
            if (index > maxIndex) {
                maxIndex = index;
            }
        }
        RobotLogger.dd(TAG, "maxIndex (targetPose): "+ maxIndex);

        //If the target pose is a previous pose in the path, we know we are at the end of the path and we no longer change the target Pose
        if (maxIndex < prevTargetIndex) {
            return prevTargetIndex;
        }
        return maxIndex;
    }

    public static int getNextTargetV5(Pose2d robotPose, double radius, PurePursuitPath path, int prevTargetIndex) {
        double minDistRadDiff = 9999.9;
        int minDistRadDiffIndex = 0;

        //Setting the bounds for all the points in the path to check (maximum bounds is +/- 50 points of the previous target
        int minBound = prevTargetIndex - 1;
        if (prevTargetIndex - 1 < 0) {
            minBound = 0;
        }
        else {
            minBound = prevTargetIndex - 1;
        }
        int maxBound;
        if (prevTargetIndex + 150 > path.path1.size() - 1) {
            maxBound = path.path1.size() - 1;
        }
        else {
            maxBound = prevTargetIndex + 150;
        }

        //GET THE MOST ACCURATE INTERCEPT IN THE BOUNDS
        for (int i=minBound; i<maxBound; i++) {
            Pose2d pathPose = new Pose2d(path.path1.get(i).x, path.path1.get(i).y, path.path1.get(i).h);  //path.path.get(i);
            double distance = Math.hypot(robotPose.getX() - pathPose.getX(), robotPose.getY() - pathPose.getY());   //Get distance between robot pose and current path pose
            double distanceRadiusDifference = Math.abs(distance - radius);

            if (distanceRadiusDifference <= minDistRadDiff) {
                minDistRadDiff = distanceRadiusDifference;
                minDistRadDiffIndex = i;
            }
        }

//        RobotLogger.dd(TAG, "minDistRadDiffIndex: "+ minDistRadDiffIndex);

        //If the target pose is a previous pose in the path, we know we are at the end of the path or that the robot is stationary.
        if (minDistRadDiffIndex <= prevTargetIndex) {
            if (prevTargetIndex != path.path1.size()-1) {   //If not the last point in the path (the robot is stationary)
                return (prevTargetIndex + 1);
            }
            return prevTargetIndex;
        }
        return minDistRadDiffIndex;
    }

    public static double AngleWrap(double angle) {
        //Ensures angle turn is between 180 and -180
        while(angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

    public static ReachedDestination reachedDestination(Pose2d robotPose, PurePursuitPathPoint pathEnd, ReachedDestination reachedDestination, double currentDuration, double pathDuration) {
        double headingDifference = Math.abs(robotPose.getHeading() - pathEnd.h);
        double locationDifference = Math.abs(Math.hypot(robotPose.getX() - pathEnd.x, robotPose.getY() - pathEnd.y));
//        RobotLogger.dd(TAG, "currentDuration: " + currentDuration);
//        RobotLogger.dd(TAG, "pathDuration: " + pathDuration);
        if (headingDifference < 0.0355 && locationDifference < 0.5 && currentDuration > (0.5*pathDuration*1000)) {
            if (reachedDestination == ReachedDestination.CHANGE || reachedDestination == ReachedDestination.TRUE) {
                return ReachedDestination.TRUE;
            }
            if (reachedDestination == ReachedDestination.FALSE) {
                RobotLogger.dd(TAG, "pathend.h: " + pathEnd.h);
                RobotLogger.dd(TAG, "robotPose.getHeading(): " + robotPose.getHeading());
                RobotLogger.dd(TAG, "headingDifference" + headingDifference);
                return ReachedDestination.CHANGE;
            }
        }
        return ReachedDestination.FALSE;
    }

}
