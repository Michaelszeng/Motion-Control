package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.teamcode.util.SafeSleep;

import java.util.ArrayList;
import java.util.List;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class Basilis<xFunction> extends LinearOpMode {
    private String TAG = "Basilis";
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    List<Pose2d> poseHistory = new ArrayList<>();
    FtcDashboard dashboard;
    double currentVelocity = 5;
    ArrayList<Double> Velocity = new ArrayList<Double>();


    final double frontToBackWheelSeparation = 18;
    final boolean mech = true;
    final double pixelToInch = 1;
    final double maxSpeed = 28;//in/sec
    final double wheelRadius = 9;//in
    final double maxWheelSpeed = 28;//ft/sec
    final double maxAcceleration = 10;//in/sec/sec
    final double maxDeceleration = 10;//in/sec/sec
    final double maxLateralAcceleration = 150; //in/sec/sec
    double [] endVCoordsX = new double[8];
    double [] endVCoordsY = new double[8];
    double [][] xFunction = new double [8][4];double [][] yFunction = new double [8][4];
    double fidelity = 100;
    double timeIncrament = 0.1;
    double q = 0;

    double w;


    public double[] redoFunction(double start, double startV, double end, double endV) {
        double [] returnCoeficients = new double [4];
        double mag = 1.5;
        returnCoeficients[3] = start;
        returnCoeficients[2] = startV * mag;
        returnCoeficients[0] = ((endV * mag) - (2*end) + (startV * mag) + (2*start));
        returnCoeficients[1] = end - returnCoeficients[3] - returnCoeficients[2] - returnCoeficients[0];
        return returnCoeficients;
    }

    public double[] interpretFunction(double t, double[] xFun, double[] yFun) {
        double [] returnCoordinates = new double [2];
        returnCoordinates[0] = xFun[0]*Math.pow(t, 3) + xFun[1]*Math.pow(t, 2) + xFun[2]*Math.pow(t, 1) + xFun[3];
        returnCoordinates[1] = yFun[0]*Math.pow(t, 3) + yFun[1]*Math.pow(t, 2) + yFun[2]*Math.pow(t, 1) + yFun[3];
        return returnCoordinates;
    }

    public double distanceCalc(double x1, double y1, double x2, double y2) {
        //System.out.println((double)Math.sqrt((double)(y2 - y1) * (double)(y2 - y1) + (double)(x2 - x1) * (double)(x2 - x1)) + " " + x1 + " " + y1 + " " + x2 + " " + y2);
        return Math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
    }
    public double graphDistance (double[] xFun, double[] yFun, int t1, int t2) {
        double[] coords1 = new double[2];
        double[] coords2 = new double[2];
        double distance = 0;
        int startTime = (t1);
        int endTime = (t2);
        for(double i = startTime; i < endTime; i ++) {
            coords1 = interpretFunction(i/fidelity, xFun,yFun);
            coords2 = interpretFunction((i+1)/fidelity, xFun,yFun);
            //System.out.println(coords1 + " " + coords2);
            distance += distanceCalc(coords1[0],coords1[1],coords2[0],coords2[1]);
            //System.out.println("Distance: " + distance + " " + startTime + " " + endTime + " " + i/fidelity);
        }
        return distance;
    }
    public double generateVelocityCurve(int z, double o, double currentSpeed) {
        double velocity = 0;
        double[] p1 = new double[2];
        double[] p2 = new double[2];
        double[] p3 = new double[2];
        Velocity.clear();
        int size;
        double time = 0;
        for (int i = 0; i < xFunction.length; i ++) {
            size = Velocity.size() -1;
            for (int j = 0; j < fidelity; j++) {
                //System.out.println("loop number 1 " + i + " " + j + " " + Velocity.size());
                if (j != 0 || i != 0) {
                    //time = (1/Velocity.get(size + j-1)) * (graphDistance(xFunction.get(i),yFunction.get(i),j-1,j)/pixelToInch);
                    time = (1/maxSpeed) * (graphDistance(xFunction[i],yFunction[i],j-1,j)/pixelToInch);


                    //velocity = Velocity.get(j-1 + size) + (time * maxAcceleration);
                    velocity = maxSpeed;


                    //Lateral Acceleration
                    if(j > 1) {
                        double a = graphDistance(xFunction[i],yFunction[i],j-1, j);
                        double b = graphDistance(xFunction[i],yFunction[i],j, j+1);
                        p1 = interpretFunction((double)(j-1)/fidelity,xFunction[i],yFunction[i]);
                        p2 = interpretFunction((double)(j+1)/fidelity,xFunction[i],yFunction[i]);
                        double c = distanceCalc(p1[0],p1[1],p2[0],p2[1]);
                        //System.out.print(a + " " + b + " " + c + " | ");
                        double angle = Math.acos((Math.pow(b,2) + Math.pow(a,2) - Math.pow(c,2))/(2*b*a));
                        angle = Math.PI - angle;
                        double lateralAcceleration = Math.sqrt((-2*Math.pow(b, 2)*Math.cos(angle)) + 2*Math.pow(b,2));
                        lateralAcceleration = lateralAcceleration/(Math.pow(time, 2) * pixelToInch);
                        //System.out.println(lateralAcceleration);
                        velocity = Math.min(maxSpeed * maxLateralAcceleration/lateralAcceleration, velocity);
                    }
                    if (true) {

                        //Calculate Wheel Speed
                        p1 = interpretFunction((double)(j-1)/fidelity,xFunction[i],yFunction[i]);
                        p2 = interpretFunction((double)(j)/fidelity,xFunction[i],yFunction[i]);
                        p3 = interpretFunction((double)(j+1)/fidelity,xFunction[i],yFunction[i]);
                        double deltaY1 = p1[1] - p2[1];
                        double deltaX1 = p1[0] - p2[0];
                        double deltaY2 = p2[1] - p3[1];
                        double deltaX2 = p2[0] - p3[0];
                        double deltaY3 = p1[1] - p2[1];
                        double deltaX3 = p1[0] - p2[0];
                        double deltaY4 = p2[1] - p3[1];
                        double deltaX4 = p2[0] - p3[0];
                        deltaY3 *= (frontToBackWheelSeparation/2 * pixelToInch)/Math.sqrt(Math.pow(deltaY1,2) + Math.pow(deltaX1,2));
                        deltaX3 *= (frontToBackWheelSeparation/2 * pixelToInch)/Math.sqrt(Math.pow(deltaY1,2) + Math.pow(deltaX1,2));
                        deltaY4 *= (frontToBackWheelSeparation/2 * pixelToInch)/Math.sqrt(Math.pow(deltaY2,2) + Math.pow(deltaX2,2));
                        deltaX4 *= (frontToBackWheelSeparation/2 * pixelToInch)/Math.sqrt(Math.pow(deltaY2,2) + Math.pow(deltaX2,2));
                        deltaY1 *= (wheelRadius * pixelToInch)/Math.sqrt(Math.pow(deltaY1,2) + Math.pow(deltaX1,2));
                        deltaX1 *= (wheelRadius * pixelToInch)/Math.sqrt(Math.pow(deltaY1,2) + Math.pow(deltaX1,2));
                        deltaY2 *= (wheelRadius * pixelToInch)/Math.sqrt(Math.pow(deltaY2,2) + Math.pow(deltaX2,2));
                        deltaX2 *= (wheelRadius * pixelToInch)/Math.sqrt(Math.pow(deltaY2,2) + Math.pow(deltaX2,2)); //frontToBackWheelSeparation
                        double wheelDist = distanceCalc(p2[0] + deltaY1 + deltaX3, p2[1] + deltaX1 + deltaY3, p3[0] + deltaY2 + deltaX4, p3[1] + deltaX2 + deltaY4);
                        wheelDist = Math.max(distanceCalc(p2[0] + deltaY1 - deltaX3, p2[1] + deltaX1 - deltaY3, p3[0] + deltaY2 - deltaX4, p3[1] + deltaX2 - deltaY4), wheelDist);
                        wheelDist = Math.max(distanceCalc(p2[0] - deltaY1 - deltaX3, p2[1] - deltaX1 - deltaY3, p3[0] - deltaY2 - deltaX4, p3[1] - deltaX2 - deltaY4), wheelDist);
                        wheelDist = Math.max(distanceCalc(p2[0] - deltaY1 + deltaX3, p2[1] - deltaX1 + deltaY3, p3[0] - deltaY2 + deltaX4, p3[1] - deltaX2 + deltaY4), wheelDist);
                        wheelDist = wheelDist/pixelToInch;
                        if (mech == true) {
                            wheelDist = wheelDist / Math.sqrt(2);
                        }
                        double wheelSpeed = wheelDist/time;
                        //System.out.println(wheelSpeed);
                        velocity = Math.min(maxSpeed * maxWheelSpeed/wheelSpeed, velocity);


                        Velocity.add(velocity);
                    }
                }
            }
        }

        Velocity.set(0, currentSpeed);
        //int w = 0;
        size = 0;

        for (int i = 0; i < xFunction.length; i ++) {
            for (int j = 1; j < fidelity; j++) {
                size ++;
                //System.out.println(size + " " + j);
                time = (1/Velocity.get(size-1)) * (graphDistance(xFunction[i],yFunction[i],j-1, j)/pixelToInch);
                if (Velocity.get(size-1) + (time * maxAcceleration) < Velocity.get(size)) {
                    Velocity.set(size , Velocity.get(size-1) + (time * maxAcceleration));
                    //System.out.print(size + " ");
                    //w ++;
                }
            }
        }

        //System.out.println(w);
        //w = 0;
        //System.out.println(Velocity.size());

        size = Velocity.size();
        Velocity.set(size-1, 5.0);

        for (int i = xFunction.length -1; i >= 0; i --) {
            for (int j = (int) (fidelity-1); j >= 0; j --) {
                size --;
                //System.out.println(size + " " + j);
                if (size > 0 && size < Velocity.size()) {
                    time = (1/Velocity.get(size)) * (graphDistance(xFunction[i],yFunction[i],j-1, j)/pixelToInch);
                    if (Velocity.get(size) + (time * maxDeceleration) < Velocity.get(size-1)) {
                        Velocity.set(size-1, (Velocity.get(size) + (time * maxDeceleration)));
                        //w ++;
                        //System.out.print(size + " ");
                    }
                }
            }
        }



        double[] Coords1 = new double [2];
        double[] Coords2 = new double [2];
        double times = 0;
        double preTimes = 0;
        double k = 0;
        double value = 0;
        w = 0;

        for (int j = 0; j < fidelity - o && times <= timeIncrament; j++) {
            preTimes = times;
            k ++;
            Coords1 = interpretFunction((double)(j + o)/(double)fidelity,xFunction[z],yFunction[z]);
            Coords2 = interpretFunction((double)(j + o + 1)/(double)fidelity,xFunction[z],yFunction[z]);
            double dist = distanceCalc(Coords1[0],Coords1[1],Coords2[0],Coords2[1]);
            value = j + o + (fidelity*z);
            if(value < Velocity.size()) {
                times += (double) (1 / Velocity.get((int) (value)) * (double) dist);
            }
            w ++;

        }
        k += o;
        if (k >= 90){
            k = 100;
        }
        return k;
    }
    public double headingDifference(double headingAngleRadians, double targetHeading){
        double headingDifference = headingAngleRadians - targetHeading;
        while (headingDifference >= Math.PI) {
            headingDifference -= Math.PI * 2;
        }// Think I found the issue; we used Math.PI and not -Math.PI
        while (headingDifference <= Math.PI * -1) {
            headingDifference += Math.PI * 2;
        }
        headingDifference *= -1;
        return headingDifference;
    }
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(100);
        long startTime = System.currentTimeMillis();
        int loops = 0;
        double FR = 0;
        double BR = 0;
        double FL = 0;
        double BL = 0;
        double pastFR = 0;
        double pastBR = 0;
        double pastFL = 0;
        double pastBL = 0;
        double heading = 0;
        double headingConstant = 0.017522383;
        double headingAngleRadians = 0;
        double headingDifference = 0;
        double startHeadingDifference = 0;
        double deltaRelX;
        double deltaRelY;
        double deltaX;
        double deltaY;
        double lastHeading = 0;
        double [] currentPos = new double [2];
        double [] p2 = new double [2];
        double [] lastPoint = new double [7];



        xFunction[ 0 ][0] = -14.0;xFunction[ 0 ][1] = 1.5;xFunction[ 0 ][2] = 91.5;xFunction[ 0 ][3] = -60.0;yFunction[ 0 ][0] = -135.0;yFunction[ 0 ][1] = 199.5;yFunction[ 0 ][2] = -46.5;yFunction[ 0 ][3] = 7.0;endVCoordsX[ 0 ] = 54;endVCoordsY[ 0 ] = -10;xFunction[ 1 ][0] = -83.0;xFunction[ 1 ][1] = 64.5;xFunction[ 1 ][2] = 52.5;xFunction[ 1 ][3] = 19.0;yFunction[ 1 ][0] = -11.5;yFunction[ 1 ][1] = 24.0;yFunction[ 1 ][2] = -52.5;yFunction[ 1 ][3] = 25.0;endVCoordsX[ 1 ] = 8;endVCoordsY[ 1 ] = -41;xFunction[ 2 ][0] = 155.0;xFunction[ 2 ][1] = -157.5;xFunction[ 2 ][2] = -67.5;xFunction[ 2 ][3] = 53.0;yFunction[ 2 ][0] = 122.0;yFunction[ 2 ][1] = -102.0;yFunction[ 2 ][2] = -39.0;yFunction[ 2 ][3] = -15.0;endVCoordsX[ 2 ] = 38;endVCoordsY[ 2 ] = 48;xFunction[ 3 ][0] = -65.5;xFunction[ 3 ][1] = 60.0;xFunction[ 3 ][2] = 82.5;xFunction[ 3 ][3] = -17.0;yFunction[ 3 ][0] = -88.0;yFunction[ 3 ][1] = 24.0;yFunction[ 3 ][2] = 123.0;yFunction[ 3 ][3] = -34.0;endVCoordsX[ 3 ] = 64;endVCoordsY[ 3 ] = -37;xFunction[ 4 ][0] = -107.0;xFunction[ 4 ][1] = 90.0;xFunction[ 4 ][2] = 6.0;xFunction[ 4 ][3] = 60.0;yFunction[ 4 ][0] = 64.5;yFunction[ 4 ][1] = -34.5;yFunction[ 4 ][2] = -93.0;yFunction[ 4 ][3] = 25.0;endVCoordsX[ 4 ] = -41;endVCoordsY[ 4 ] = -17;xFunction[ 5 ][0] = 6.5;xFunction[ 5 ][1] = 76.5;xFunction[ 5 ][2] = -135.0;xFunction[ 5 ][3] = 49.0;yFunction[ 5 ][0] = -24.5;yFunction[ 5 ][1] = 3.0;yFunction[ 5 ][2] = 31.5;yFunction[ 5 ][3] = -38.0;endVCoordsX[ 5 ] = 22;endVCoordsY[ 5 ] = -52;xFunction[ 6 ][0] = -16.0;xFunction[ 6 ][1] = -28.5;xFunction[ 6 ][2] = 37.5;xFunction[ 6 ][3] = -3.0;yFunction[ 6 ][0] = 13.0;yFunction[ 6 ][1] = 6.0;yFunction[ 6 ][2] = -36.0;yFunction[ 6 ][3] = -28.0;endVCoordsX[ 6 ] = -55;endVCoordsY[ 6 ] = -35;xFunction[ 7 ][0] = -7.5;xFunction[ 7 ][1] = 48.0;xFunction[ 7 ][2] = -67.5;xFunction[ 7 ][3] = -10.0;yFunction[ 7 ][0] = 2.5;yFunction[ 7 ][1] = 13.5;yFunction[ 7 ][2] = 15.0;yFunction[ 7 ][3] = -45.0;endVCoordsX[ 7 ] = -33;endVCoordsY[ 7 ] = 19;



        double pixelToInch = 13.8888888889;
        double t = 0;
        int l = 0;
        double targetHeading = Math.atan2(p2[1] - currentPos[1], p2[0] - currentPos[0]);
        currentPos[0] = (xFunction[l][0]*Math.pow((t/100),3) + xFunction[l][1]*Math.pow((t/100),2) + xFunction[l][2]*(t/100) + xFunction[l][3]);
        currentPos[1] = (yFunction[l][0]*Math.pow((t/100),3) + yFunction[l][1]*Math.pow((t/100),2) + yFunction[l][2]*(t/100) + yFunction[l][3]);
        t += 1;
        p2[0] = (xFunction[l][0]*Math.pow((t/100),3) + xFunction[l][1]*Math.pow((t/100),2) + xFunction[l][2]*(t/100) + xFunction[l][3]);
        p2[1] = (yFunction[l][0]*Math.pow((t/100),3) + yFunction[l][1]*Math.pow((t/100),2) + yFunction[l][2]*(t/100) + yFunction[l][3]);
        double startingHeading = targetHeading;

        double maxHeadingDifference = Math.PI * 2;
        boolean loop = true;
        double powerRatio = 1;



        while (! isStopRequested()) {


            if (loop) {
                loops ++;
                int i = 0;
                int j = 0;
                drive.setMotorPowers(FR, FL, BR, BL);
                List<Double> wheelPositions = drive.getWheelPositions();
                ;
                RobotLogger.dd(TAG, wheelPositions.toString());
                SafeSleep.sleep_milliseconds(this, 10);


                targetHeading = Math.atan2(p2[1] - currentPos[1], p2[0] - currentPos[0]);
                heading = ((wheelPositions.get(2) - wheelPositions.get(1)) + (wheelPositions.get(0) - wheelPositions.get(3)));
                headingAngleRadians = heading * headingConstant + startingHeading;
                headingDifference = headingDifference(headingAngleRadians,targetHeading);

                double dist = Math.sqrt(Math.pow(p2[0]-currentPos[0],2) + (Math.pow(p2[1]-currentPos[1],2)));
                //maxDist = Math.max(maxDist,dist);
                maxHeadingDifference = Math.min(Math.abs(maxHeadingDifference), Math.abs(headingDifference));
                maxHeadingDifference = Math.max(Math.PI/4,maxHeadingDifference);
                double r = (headingDifference / maxHeadingDifference);
                double k = dist/8;
                k = Math.min(1, k);
                k *= powerRatio;
                FR = (r) + k;
                BR = (r) + k;
                FL = (r * -1) + k;
                BL = (r * -1) + k;
                double power = Math.max( Math.max ( Math.abs(FR) ,Math.abs(BL) ),Math.max( Math.abs(BR), Math.abs(FL) ) );
                if (power > 1) {
                    FR = FR / power;
                    BR = BR / power;
                    FL = FL / power;
                    BL = BL / power;
                }

                if (Math.abs(startHeadingDifference) + Math.PI/6 < Math.abs(headingDifference)) {
                    double finalX = (xFunction[l][0] + xFunction[l][1] + xFunction[l][2] + xFunction[l][3]);
                    double finalY = (yFunction[l][0] + yFunction[l][1] + yFunction[l][2] + yFunction[l][3]);
                    double magni = Math.sqrt(Math.pow(currentPos[0] - finalX, 2) + Math.pow(currentPos[1] - finalY, 2));
                    magni *= 0.25;
                    xFunction[l] = redoFunction(currentPos[0], magni * Math.cos(headingAngleRadians), finalX, endVCoordsX[l] - finalX);
                    yFunction[l] = redoFunction(currentPos[1], magni * Math.sin(headingAngleRadians), finalY, endVCoordsY[l] - finalY);
                    t = 0;
                    p2[0] = (xFunction[l][0]*Math.pow((t/100),3) + xFunction[l][1]*Math.pow((t/100),2) + xFunction[l][2]*(t/100) + xFunction[l][3]);
                    p2[1] = (yFunction[l][0]*Math.pow((t/100),3) + yFunction[l][1]*Math.pow((t/100),2) + yFunction[l][2]*(t/100) + yFunction[l][3]);
                    targetHeading = Math.atan2(p2[1] - currentPos[1], p2[0] - currentPos[0]);
                    startHeadingDifference = headingDifference(headingAngleRadians,targetHeading);
                }


                double deltaFR = wheelPositions.get(0) - pastFR;
                double deltaBR = wheelPositions.get(2) - pastBR;
                double deltaFL = wheelPositions.get(1) - pastFL;
                double deltaBL = wheelPositions.get(3) - pastBL;
                double averageHeading = (headingAngleRadians + lastHeading) / 2;
                deltaRelX = ((1 * deltaFL) - deltaFR - deltaBL + deltaBR) / 4;
                deltaRelY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4;
                deltaX = (deltaRelX * Math.sin(averageHeading)) + (deltaRelY * Math.cos(averageHeading));
                deltaY = (deltaRelX * Math.cos(averageHeading) * -1) + (deltaRelY * Math.sin(averageHeading));
                currentPos[0] += deltaX;
                currentPos[1] += deltaY;

                double offset = 2;
                if ((currentPos[0] > p2[0] - offset && currentPos[0] < p2[0] + offset) && (currentPos[1] > p2[1] - offset && currentPos[1] < p2[1] + offset)){

                    /*
                    if (q == 50) {
                        q = 0;
                        t = 0;
                        xFunction[l] = redoFunction(currentPos[0], magni * Math.cos(headingAngleRadians), finalX, endVCoordsX[l] - finalX);
                        yFunction[l] = redoFunction(currentPos[1], magni * Math.sin(headingAngleRadians), finalY, endVCoordsY[l] - finalY);
                    }

                     */
                    t = generateVelocityCurve(l,t,currentVelocity);//currentVelocity needs to be added
                    //magni * Math.sin(headingAngleRadians)
                    lastPoint[0] = p2[0];
                    lastPoint[1] = p2[0];
                    p2[0] = (xFunction[l][0]*Math.pow((t/100),3) + xFunction[l][1]*Math.pow((t/100),2) + xFunction[l][2]*(t/100) + xFunction[l][3]);
                    p2[1] = (yFunction[l][0]*Math.pow((t/100),3) + yFunction[l][1]*Math.pow((t/100),2) + yFunction[l][2]*(t/100) + yFunction[l][3]);
                    targetHeading = Math.atan2(p2[1] - currentPos[1], p2[0] - currentPos[0]);
                    startHeadingDifference = headingDifference(headingAngleRadians,targetHeading);
                    if (l * fidelity + t < Velocity.size()) {
                        currentVelocity = Velocity.get((int) (l * fidelity + t));
                    }
                    else {
                        currentVelocity = Velocity.get(Velocity.size()-1);
                    }
                    powerRatio = currentVelocity/maxSpeed;
                    powerRatio = Math.min(powerRatio, 1);
                    powerRatio = Math.max(powerRatio, 0.5);
                    if (t >= 100){
                        l ++;
                        t = 0;
                    }
                    if (l >= xFunction.length){
                        loop = false;
                    }

                }


                TelemetryPacket packet = new TelemetryPacket();
                Canvas fieldOverlay = packet.fieldOverlay();
                Pose2d currentPose = new Pose2d(-1 * currentPos[1], -1 * currentPos[0], ((Math.PI*3)/2 - headingAngleRadians));
                poseHistory.add(currentPose);
                fieldOverlay.setStrokeWidth(1);
                fieldOverlay.setStroke("#3F51B5");
                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
                DashboardUtil.drawRobot(fieldOverlay, currentPose);
                packet.put("mode", SampleMecanumDrive.Mode.FOLLOW_TRAJECTORY);
                packet.put("currentVelocity", currentVelocity);
                packet.put("l", l);
                packet.put("t", t);
                packet.put("w", w);
                packet.put("size", Velocity.size());
                packet.put("powerRatio", powerRatio);
                packet.put("heading", headingAngleRadians);
                dashboard.sendTelemetryPacket(packet);

                pastFR = wheelPositions.get(0);
                pastBR = wheelPositions.get(2);
                pastFL = wheelPositions.get(1);
                pastBL = wheelPositions.get(3);
                lastHeading = headingAngleRadians;
            }
        }
    }
}

