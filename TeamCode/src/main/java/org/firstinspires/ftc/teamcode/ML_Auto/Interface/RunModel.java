package org.firstinspires.ftc.teamcode.ML_Auto.Interface;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.CNN.CNN;
import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.KNN.KNN;
import org.firstinspires.ftc.teamcode.ML_Auto.MLConstants;
import org.firstinspires.ftc.teamcode.ML_Auto.Utils.FileWriter;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.ArrayList;

/*
 *  Note: Only KNN is currently functional (but untested on real robot)
 *  This program will allow a ML model to drive the robot autonomously.
 */

@Config
@Autonomous(group = "ML_Auto")
public class RunModel extends LinearOpMode {
    private AllHardwareMap hm;

    //TODO Tune robot movement constraints here
    private static final double ACCEPTABLE_MOVEMENT_PERCENT_ERROR = 0.1;
    private static final double[] CORRECTION_POWER_CONSTRAINTS = new double[] { 0.3, 0.7 };
    SampleMecanumDrive _drive;

    public void runOpMode(){
        _drive = new SampleMecanumDrive(hardwareMap);

        hm = new AllHardwareMap(hardwareMap, _drive);
        hm.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hm.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hm.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hm.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //TODO Reverse direction of necessary motors
        hm.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hm.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        switch (MLConstants.USED_MODEL){
            case CNN:
                CNN model = (CNN) FileWriter.getSerializedObject(MLConstants.CNN_MODEL_FILEPATH);

                waitForStart();

                long init = System.currentTimeMillis();
                double[][] out = model.classify(new double[][]{{System.currentTimeMillis() - init}});

                break;
            case KNN:
                ArrayList<KNN> KNN_MODELS = (ArrayList<KNN>) FileWriter.getSerializedObject(MLConstants.KNN_MODEL_FILEPATH);

                if(KNN_MODELS == null) {
                    telemetry.addData("MODEL TYPE", MLConstants.USED_MODEL.toString());
                    telemetry.addData("MODEL STATUS", "File Missing!");
                } else {
                    telemetry.addData("MODEL TYPE", MLConstants.USED_MODEL.toString());
                    telemetry.addData("MODEL STATUS", "Ready to Run!");
                }
                telemetry.update();

                waitForStart();

                init = System.currentTimeMillis();

                if(KNN_MODELS == null)
                    throw new NullPointerException("An unknown error occurred when loading model. " +
                            MLConstants.USED_MODEL.toString() + " model file is missing or corrupted!");

                while(opModeIsActive()) {
                    ArrayList<Boolean> done = new ArrayList<>();

                    Thread fl = new Thread(){
                        public void run(){
                            double encoder = KNN_MODELS.get(0).classify(new double[]{ System.currentTimeMillis() - init});
                            double power = KNN_MODELS.get(1).classify(new double[]{ System.currentTimeMillis() - init});
                            runToEncoderPosition(hm.frontLeft, power, encoder);
                            done.add(true);
                        }
                    };

                    Thread fr = new Thread(){
                        public void run(){
                            double encoder = KNN_MODELS.get(2).classify(new double[]{ System.currentTimeMillis() - init});
                            double power = KNN_MODELS.get(3).classify(new double[]{ System.currentTimeMillis() - init});
                            runToEncoderPosition(hm.frontRight, power, encoder);
                            done.add(true);
                        }
                    };

                    Thread bl = new Thread(){
                        public void run(){
                            double encoder = KNN_MODELS.get(4).classify(new double[]{ System.currentTimeMillis() - init});
                            double power = KNN_MODELS.get(5).classify(new double[]{ System.currentTimeMillis() - init});
                            runToEncoderPosition(hm.backLeft, power, encoder);
                            done.add(true);
                        }
                    };

                    Thread br = new Thread(){
                        public void run(){
                            double encoder = KNN_MODELS.get(6).classify(new double[]{ System.currentTimeMillis() - init});
                            double power = KNN_MODELS.get(7).classify(new double[]{ System.currentTimeMillis() - init});
                            runToEncoderPosition(hm.backRight, power, encoder);
                            done.add(true);
                        }
                    };

                    fl.start();
                    fr.start();
                    bl.start();
                    br.start();

                    while(done.size() < 4)
                        try{
                            Thread.sleep(3);
                        } catch (InterruptedException e){
                            e.printStackTrace();
                        }
                }
                break;
            case LogisticRegression:
                waitForStart();
                break;
        }
    }

    private void runToEncoderPosition(DcMotorEx motor, double power, double encoderPosition){
        int currPos = motor.getCurrentPosition();

        double powRatio = (currPos - (int) encoderPosition) / encoderPosition > 0.3 ? 1 / 2d : 1 / 3d;
        if((currPos - (int) encoderPosition > 0 && power < 0) || (currPos - (int) encoderPosition < 0 && power > 0))
            power = -(power * powRatio);

        if(currPos - (int) encoderPosition == 0)
            power = 0;

        double percentError = (currPos - (int) encoderPosition) / encoderPosition;
        if(percentError > ACCEPTABLE_MOVEMENT_PERCENT_ERROR && Math.abs(power) < 0.05){
            if(percentError > CORRECTION_POWER_CONSTRAINTS[1])
                percentError = CORRECTION_POWER_CONSTRAINTS[1];

            double conversionUnit = (CORRECTION_POWER_CONSTRAINTS[1] - CORRECTION_POWER_CONSTRAINTS[0]) /
                    (CORRECTION_POWER_CONSTRAINTS[1] - ACCEPTABLE_MOVEMENT_PERCENT_ERROR);
            power = CORRECTION_POWER_CONSTRAINTS[0] + conversionUnit * (CORRECTION_POWER_CONSTRAINTS[1] - percentError);
        }

        motor.setPower(power);
    }
}
