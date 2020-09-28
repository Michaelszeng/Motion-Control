package org.firstinspires.ftc.teamcode.drive;

import android.util.Log;

import java.util.ArrayList;

public class ThreeOdometerLocalizer {
    private String TAG = "ThreeOdometerLocalizer";

    public static double odoTicksPerRevRight;
    public static double odoTicksPerRevLeft;
    public static double odoTicksPerRevHorizontal;
    public static double odoWheelRadius;
    public static double ODOMETRY_TRACK_WIDTH;
    public static double ODOMETRY_HORIZONTAL_TRACK_WIDTH;

    public ThreeOdometerLocalizer() {
        this.odoTicksPerRevRight = DriveConstants.odoTicksPerRevRight;
        this.odoTicksPerRevLeft = DriveConstants.odoTicksPerRevLeft;
        this.odoTicksPerRevHorizontal = DriveConstants.odoTicksPerRevHorizontal;
        this.odoWheelRadius = DriveConstants.odoWheelRadius;
        this.ODOMETRY_TRACK_WIDTH = DriveConstants.ODOMETRY_TRACK_WIDTH;
        this.ODOMETRY_HORIZONTAL_TRACK_WIDTH = DriveConstants.ODOMETRY_HORIZONTAL_TRACK_WIDTH;
    }

    //NON-FUNCTIONAL
    //Inspiration: https://drive.google.com/file/d/11c3Z9EkDj2_GuOQSFzpVzba_HPKqD6ej/view
    public ArrayList<Double> getPoseChangeV1(double leftChangeTicks, double rightChangeTicks, double horizontalChangeTicks, double headingCurrentDeg, double headingPrevDeg) {  //heading in deg
        double leftChange = ticksToInches((int) leftChangeTicks, odoTicksPerRevLeft);
        double rightChange = ticksToInches((int) rightChangeTicks, odoTicksPerRevRight);
        double horizontalChange = ticksToInches((int) horizontalChangeTicks, odoTicksPerRevHorizontal);
        double headingCurrent = degreesToRadians(headingCurrentDeg);
        double headingPrev = degreesToRadians(headingPrevDeg);

//        double ENCODER_COUNTS_PER_INCH_LEFT = (1 / odoTicksPerRevLeft) * (2 * Math.PI * odoWheelRadius);
//        double ENCODER_COUNTS_PER_INCH_RIGHT = (1 / odoTicksPerRevRight) * (2 * Math.PI * odoWheelRadius);
//        double odoHeadingChange = ((rightChangeTicks) / (ODOMETRY_TRACK_WIDTH * ENCODER_COUNTS_PER_INCH_RIGHT)) - ((leftChangeTicks) / (ODOMETRY_TRACK_WIDTH * ENCODER_COUNTS_PER_INCH_LEFT));
        double odoHeadingChange = (rightChange - leftChange) / ODOMETRY_TRACK_WIDTH;

        Log.d(TAG, "odoHeadingChange: " + odoHeadingChange);
//        Log.d(TAG, "IMUHeadingChange: " + (headingCurrent - headingPrev));

        //these changes are still perpendicular to the robot, do they need to be rotated based on the robot's heading
        double xChange;
        double yChange;

        if (odoHeadingChange == 0 || rightChange == leftChange) {     //if heading change is very small, radius becomes infinite, can't do that
            xChange = horizontalChange;
            yChange = (leftChange + rightChange) / 2;
        }
        else {
            double longitudinalRadius = (ODOMETRY_TRACK_WIDTH * 0.5 * (leftChange + rightChange)) / (rightChange - leftChange);
            double lateralRadius = (horizontalChange / odoHeadingChange) - Math.abs(ODOMETRY_HORIZONTAL_TRACK_WIDTH);
            Log.d(TAG, "Longitudinal Radius: " + longitudinalRadius);
            Log.d(TAG, "Lateral Radius: " + lateralRadius);

            xChange = longitudinalRadius * (Math.cos(odoHeadingChange) - 1) + lateralRadius * Math.sin(odoHeadingChange);
            yChange = longitudinalRadius * Math.sin(odoHeadingChange) + lateralRadius * (1 - Math.cos(odoHeadingChange));
        }

        //rotating the local xChange and yChange to global
        double globalHeadingAvg = -(headingCurrent + headingPrev) / 2;
        double globalVectorX = xChange * Math.cos(-globalHeadingAvg) + yChange * Math.sin(-globalHeadingAvg);
        double globalVectorY =  - xChange * Math.sin(-globalHeadingAvg) + yChange * Math.cos(-globalHeadingAvg);

        ArrayList<Double> results = new ArrayList<>();
        results.add(globalVectorX);
        results.add(globalVectorY);

        return results;
    }

    //FUNCTIONAL
    //Inspiration: https://www.youtube.com/watch?v=ZW7T6EFyYnc&t=330s
    public ArrayList<Double> getPoseChangeV2(double leftChangeTicks, double rightChangeTicks, double horizontalChangeTicks, double headingCurrent, double headingPrev) {  //heading in deg, clockwise = positive
        //Local = of the field
        //Global = of the robot

        double leftChange = ticksToInches((int) leftChangeTicks, odoTicksPerRevLeft);
        double rightChange = ticksToInches((int) rightChangeTicks, odoTicksPerRevRight);

        double headingChange = degreesToRadians(headingCurrent - headingPrev);
        double odoHeadingChange = (leftChange - rightChange) / (ODOMETRY_TRACK_WIDTH);
        //need to subtract (strafeWheelTrackWidth * globalHeading) so the robot doesn't think its moving when it turns
        double horizontalChange = ticksToInches((int) horizontalChangeTicks, odoTicksPerRevHorizontal) - (headingChange * ODOMETRY_HORIZONTAL_TRACK_WIDTH);
        Log.d(TAG, "subtracted from strafe: " + (headingChange * ODOMETRY_HORIZONTAL_TRACK_WIDTH));

        double localVectorX = horizontalChange;
        double localVectorY = (leftChange + rightChange) / 2;

        double globalHeadingAvg = degreesToRadians((headingCurrent + headingPrev) / 2);

        double globalVectorX = localVectorX * Math.cos(globalHeadingAvg) + localVectorY * Math.sin(globalHeadingAvg);
        double globalVectorY =  - localVectorX * Math.sin(globalHeadingAvg) + localVectorY * Math.cos(globalHeadingAvg);

        ArrayList<Double> results = new ArrayList<>();
        results.add(globalVectorX);
        results.add(globalVectorY);
        return results;
    }

    //FUNCTIONAL
    //Inspiration: Recharged Green (https://github.com/RechargedGreen/SkystoneApp/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/odometry/ThreeWheel.kt)
    public ArrayList<Double> getPoseChangeV3(double leftChangeTicks, double rightChangeTicks, double horizontalChangeTicks, double currentLeftTicks, double currentRightTicks, double currentHorizontalTicks, double headingCurrent, double headingPrev) {  //heading change in radians, clockwise = positive
        //Identical to V2 but uses latest/current heading instead of the avg heading
        //Local = of the field
        //Global = of the robot

        double leftChange = ticksToInches((int) leftChangeTicks, odoTicksPerRevLeft);
        double rightChange = ticksToInches((int) rightChangeTicks, odoTicksPerRevRight);
        double horizontalChange = ticksToInches((int) horizontalChangeTicks, odoTicksPerRevHorizontal);
        double currentLeft = ticksToInches((int) currentLeftTicks, odoTicksPerRevLeft);
        double currentRight = ticksToInches((int) currentRightTicks, odoTicksPerRevRight);

        double odoHeadingChange = (leftChange - rightChange) / ODOMETRY_TRACK_WIDTH;
        double lastRawAngle = (currentLeft - currentRight) / ODOMETRY_TRACK_WIDTH;
        //Logs need to be very similar
//        Log.d(TAG, "lastRawAngleOdo: " + lastRawAngle);
//        Log.d(TAG, "IMU            : " + degreesToRadians(headingCurrent));

        double horizontalReadingFromRotation = odoHeadingChange * ODOMETRY_HORIZONTAL_TRACK_WIDTH;

        double localVectorY = (leftChange + rightChange) / 2.0;
        double localVectorX = horizontalChange - horizontalReadingFromRotation;

        double globalVectorX = localVectorX * Math.cos(lastRawAngle) + localVectorY * Math.sin(lastRawAngle);
        double globalVectorY =  - localVectorX * Math.sin(lastRawAngle) + localVectorY * Math.cos(lastRawAngle);

        ArrayList<Double> results = new ArrayList<>();
        results.add(globalVectorX);
        results.add(globalVectorY);
        return results;
    }



    public double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    public double ticksToInches(int ticks, double odoTicksPerRev) {
        double revolutions = ticks/odoTicksPerRev;
        double inches = revolutions * Math.PI * 2 * odoWheelRadius;
        return inches;
    }

    public double normalizeRadians(double angle){
        while(angle >= 2*Math.PI) {
            angle -= 2*Math.PI;
        }
        while(angle < 0.0) {
            angle += 2*Math.PI;
        }
        return angle;
    }

}
