package org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.KNN;

import java.util.Arrays;

//simple class to model instances (features + class)
public class KNNData {
    public double[] x;
    public double label;
    public KNNData(double[] x, double label){
        this.x = x;
        this.label = label;
    }

    @Override
    public String toString(){
        return "{" + Arrays.toString(x) + ", " + label + "}";
    }
}
