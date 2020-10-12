package org.firstinspires.ftc.teamcode.ML_Auto.Interface;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.CNN.CNN;
import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.CNN.CNNData;
import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.KNN.KNN;
import org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.KNN.KNNData;
import org.firstinspires.ftc.teamcode.ML_Auto.MLConstants;
import org.firstinspires.ftc.teamcode.ML_Auto.Utils.FileWriter;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

/*
 *  Note: Only KNN is currently functional (but untested on real robot)
 *  This program will allow the user to train the ML model on the Robot Controller phone.
 */

@Config
@Autonomous(group = "ML_Auto")
public class TrainModel extends LinearOpMode {
    public void runOpMode() {
        waitForStart();

        if (opModeIsActive()) {
            switch (MLConstants.USED_MODEL) {
                case CNN:
                    List<CNNData> data = new ArrayList<>();

                    try {
                        Scanner scan = new Scanner(new File(MLConstants.TRAINING_DATA_FILEPATH));
                        while (scan.hasNextLine()) {
                            double[] x = new double[]{scan.nextDouble()};
                            double[] y = new double[8];
                            for (int i = 0; i < 8; i++)
                                y[i] = scan.nextDouble();
                            data.add(new CNNData(x, y));
                        }
                        scan.close();
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                    }

                    telemetry.addData("TRAINING_STATUS", "Training...");
                    telemetry.update();

                    CNN cnn = new CNN(CNN.DEFAULT_M, CNN.DEFAULT_NODES, CNN.DEFAULT_ITERATIONS);
                    cnn.train(data);
                    FileWriter.writeSerializedObject(MLConstants.CNN_MODEL_FILEPATH, cnn);
                    break;
                case KNN:
                    ArrayList<KNNData> frontLeftEncoder = new ArrayList<>();
                    ArrayList<KNNData> frontLeftPower = new ArrayList<>();
                    ArrayList<KNNData> frontRightEncoder = new ArrayList<>();
                    ArrayList<KNNData> frontRightPower = new ArrayList<>();
                    ArrayList<KNNData> backLeftEncoder = new ArrayList<>();
                    ArrayList<KNNData> backLeftPower = new ArrayList<>();
                    ArrayList<KNNData> backRightEncoder = new ArrayList<>();
                    ArrayList<KNNData> backRightPower = new ArrayList<>();

                    try {
                        Scanner scan = new Scanner(new File(MLConstants.TRAINING_DATA_FILEPATH));
                        while (scan.hasNextLine() && opModeIsActive()) {
                            double[] x = new double[]{scan.nextDouble()};

                            frontLeftEncoder.add(new KNNData(x, scan.nextDouble()));
                            frontLeftPower.add(new KNNData(x, scan.nextDouble()));
                            frontRightEncoder.add(new KNNData(x, scan.nextDouble()));
                            frontRightPower.add(new KNNData(x, scan.nextDouble()));
                            backLeftEncoder.add(new KNNData(x, scan.nextDouble()));
                            backLeftPower.add(new KNNData(x, scan.nextDouble()));
                            backRightEncoder.add(new KNNData(x, scan.nextDouble()));
                            backRightPower.add(new KNNData(x, scan.nextDouble()));
                        }
                        scan.close();
                    } catch (FileNotFoundException e) {
                        e.printStackTrace();
                        telemetry.addData("TRAINING_STATUS", "Failed to read training data!");
                        telemetry.update();
                        break;
                    }

                    ArrayList<KNN> KNN_MODELS = new ArrayList<>();

                    if (opModeIsActive())
                        for (int i = 0; i < 8; i++)
                            KNN_MODELS.add(new KNN(KNN.DEFAULT_K));

                    telemetry.addData("TRAINING_STATUS", "Training...");
                    telemetry.update();

                    if (opModeIsActive()) {
                        KNN_MODELS.get(0).train(frontLeftEncoder);
                        KNN_MODELS.get(1).train(frontLeftPower);
                        KNN_MODELS.get(2).train(frontRightEncoder);
                        KNN_MODELS.get(3).train(frontRightPower);
                        KNN_MODELS.get(4).train(backLeftEncoder);
                        KNN_MODELS.get(5).train(backLeftPower);
                        KNN_MODELS.get(6).train(backRightEncoder);
                        KNN_MODELS.get(7).train(backRightPower);

                        FileWriter.writeSerializedObject(MLConstants.KNN_MODEL_FILEPATH, KNN_MODELS);
                    }

                    while (opModeIsActive()) {
                        telemetry.addData("TRAINING_STATUS", "Training Complete");
                        telemetry.update();
                    }
                    break;
                case LogisticRegression:
                    break;
            }
        }

        while (opModeIsActive()) {
            telemetry.addData("TRAINING_STATUS", "DONE");
            telemetry.update();
        }
    }
}
