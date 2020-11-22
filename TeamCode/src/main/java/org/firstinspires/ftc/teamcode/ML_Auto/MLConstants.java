package org.firstinspires.ftc.teamcode.ML_Auto;

import android.os.Environment;

import org.firstinspires.ftc.teamcode.ML_Auto.Interface.ModelType;

public class MLConstants {
    public static final String TRAINING_DATA_FILEPATH = Environment.getExternalStorageDirectory().getPath() + "/FTC/ML Autonomous/AutonomousTrainingData.csv";
    public static final String ROOT_FILEPATH = Environment.getExternalStorageDirectory().getPath() + "/FTC/ML Autonomous/";
    public static final String KNN_MODEL_FILEPATH = Environment.getExternalStorageDirectory().getPath() + "/FTC/ML Autonomous/KNN_Model.mdl";
    public static final String CNN_MODEL_FILEPATH = Environment.getExternalStorageDirectory().getPath() + "/FTC/ML Autonomous/CNN_Model.mdl";
    public static final String LOG_REG_MODEL_FILEPATH = Environment.getExternalStorageDirectory().getPath() + "/FTC/ML Autonomous/LogReg_Model.mdl";
    public static final ModelType USED_MODEL = ModelType.KNN;
}
