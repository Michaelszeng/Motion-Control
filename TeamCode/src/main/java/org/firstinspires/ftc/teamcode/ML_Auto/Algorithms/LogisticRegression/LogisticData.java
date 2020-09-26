package org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.LogisticRegression;

import java.util.Arrays;

public class LogisticData {
    public int label;
    public int[] x;

    public LogisticData(int label, int[] x) {
        this.label = label;
        this.x = x;
    }

    @Override
    public String toString(){
        return "{" + Arrays.toString(x) + ", " + label + "}";
    }
}
