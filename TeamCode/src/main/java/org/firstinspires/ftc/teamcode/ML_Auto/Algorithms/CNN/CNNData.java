package org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.CNN;

import java.util.Arrays;

public class CNNData {
    public double[] x;
    public double[] label;

    public CNNData(double[] x, double[] label){
        this.x = x;
        this.label = label;
    }

    @Override
    public String toString(){
        return "{" + Arrays.toString(x) + ", " + Arrays.toString(label) + "}";
    }
}
