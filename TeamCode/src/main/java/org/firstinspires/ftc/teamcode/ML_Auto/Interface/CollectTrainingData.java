package org.firstinspires.ftc.teamcode.ML_Auto.Interface;

import android.os.CountDownTimer;
import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ML_Auto.MLConstants;
import org.firstinspires.ftc.teamcode.ML_Auto.Utils.FileWriter;
import org.firstinspires.ftc.teamcode.ML_Auto.Utils.FourWheelMecanumDrivetrain;
import org.firstinspires.ftc.teamcode.util.AllHardwareMap;

import java.util.ArrayList;

/*
 *  Note: Only KNN is currently functional (but untested on real robot)
 *  This program will allow the user to collect training data from multiple runs.
 */

@Config
@Autonomous(group = "ML_Auto")
public class CollectTrainingData extends LinearOpMode {
    private ArrayList<Double> data = new ArrayList<>();
    private AllHardwareMap hm;
    private FourWheelMecanumDrivetrain drivetrain;

    public void runOpMode() {
        hm = new AllHardwareMap(hardwareMap);
        drivetrain = new FourWheelMecanumDrivetrain(hm);

        boolean append = true;

        while (!isStarted()){
            if(gamepad1.a)
                append = true;
            else if(gamepad1.b)
                append = false;

            telemetry.addData("File Write Status", append ? "APPEND" : "OVERWRITE");
            telemetry.update();
        }

        waitForStart();

        teleopMvmt();

        while (opModeIsActive()){
            if(hm.frontLeft.getVelocity() != 0.0 || hm.frontRight.getVelocity() != 0.0 || hm.backLeft.getVelocity() != 0.0
                    || hm.backRight.getVelocity() != 0.0)
                break;
        }

        telemetry.addData("Save Training Data Info", "Press A to save. Stop the program to cancel.");
        telemetry.addData("Time Remaining in Autonomous", "30 secs");
        telemetry.update();

        CountDownTimer countDownTimer = new CountDownTimer(30000, 1000) {
            @Override
            public void onTick(long millisUntilFinished) {
                telemetry.addData("Save Training Data Info", "Press A to save. Stop the program to cancel.");
                telemetry.addData("Time Remaining in Autonomous", millisUntilFinished / 1000 + " secs");
                telemetry.update();
            }

            @Override
            public void onFinish() {
                //mTextField.setText("done!");
            }
        };
        countDownTimer.start();

        long init = System.currentTimeMillis();
        while (opModeIsActive()) {
            data.add((double) (System.currentTimeMillis() - init) /*/ 1000d*/);
            data.add((double) hm.frontLeft.getCurrentPosition());
            data.add(hm.frontLeft.getPower());
            data.add((double) hm.frontRight.getCurrentPosition());
            data.add(hm.frontRight.getPower());
            data.add((double) hm.backLeft.getCurrentPosition());
            data.add(hm.backLeft.getPower());
            data.add((double) hm.backRight.getCurrentPosition());
            data.add(hm.backRight.getPower());

            if(gamepad1.a) {
                for (int i = 0; i < data.size(); i += 1)
                    FileWriter.writeFile(MLConstants.TRAINING_DATA_FILEPATH, data.get(i) + "\n", i != 0 || append);
                break;
            }
        }
    }

    private void teleopMvmt() {
        Thread t = new Thread() {
            public void run() {
                while(opModeIsActive()) {
                    double turn = (-1) * (gamepad1.left_trigger - gamepad1.right_trigger) * 1;

                    if (!(gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0 && turn == 0)) {

                        double speed;

                        if (gamepad1.left_stick_x == 0 && gamepad1.right_stick_y == 0) {
                            speed = 0;
                        } else if (gamepad1.right_stick_y == 0) {
                            speed = Math.sqrt(2) * Math.abs(gamepad1.left_stick_x);
                        } else if (gamepad1.left_stick_x == 0) {
                            speed = Math.abs(gamepad1.right_stick_y);
                        } else {
                            speed = Math.min(Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_y), 1);
                        }

                        double angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.right_stick_y);
                        drivetrain.MoveAngle(speed, angle, turn, false);
                    } else {
                        drivetrain.stop();
                    }
                }
            }

        };
        t.start();
    }
}
