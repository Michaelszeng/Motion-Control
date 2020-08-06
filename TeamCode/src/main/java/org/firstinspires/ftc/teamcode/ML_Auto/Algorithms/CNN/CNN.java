package org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.CNN;

import java.io.Serializable;
import java.util.Arrays;
import java.util.List;

public class CNN implements Serializable {
    private static final long serialVersionUID = 1L;

    public static final int DEFAULT_M = 4;
    public static final int DEFAULT_NODES = 400;
    public static final int DEFAULT_ITERATIONS = 10000;

    private int m;
    private int nodes;
    private int iterations;

    private double[][] W1;
    private double[][] b1;

    private double[][] W2;
    private double[][] b2;

    public CNN(int m, int nodes, int iterations){
        this.m = m;
        this.nodes = nodes;
        this.iterations = iterations;
    }

    public void train(List<CNNData> data){
        double[][] X = new double[data.size()][1];
        double[][] Y = new double[data.size()][data.get(0).x.length];

        for(int i = 0; i < data.size(); i++){
            for(int j = 0; j < data.get(i).x.length; j++)
                X[i][j] = data.get(i).x[j];

            for(int j = 0; j < data.get(i).label.length; j++)
                Y[i][j] = data.get(i).label[j];
        }

        X = MatrixOperations.T(X);
        Y = MatrixOperations.T(Y);

        W1 = MatrixOperations.random(nodes, 2);
        b1 = new double[nodes][m];

        W2 = MatrixOperations.random(1, nodes);
        b2 = new double[1][m];

        for (int i = 0; i < iterations; i++) {
            // Foward Prop
            // LAYER 1
            double[][] Z1 = MatrixOperations.add(MatrixOperations.dot(W1, X), b1);
            double[][] A1 = MatrixOperations.sigmoid(Z1);

            //LAYER 2
            double[][] Z2 = MatrixOperations.add(MatrixOperations.dot(W2, A1), b2);
            double[][] A2 = MatrixOperations.sigmoid(Z2);

            double cost = MatrixOperations.cross_entropy(m, Y, A2);
            //costs.getData().add(new XYChart.Data(i, cost));

            // Back Prop
            //LAYER 2
            double[][] dZ2 = MatrixOperations.subtract(A2, Y);
            double[][] dW2 = MatrixOperations.divide(MatrixOperations.dot(dZ2, MatrixOperations.T(A1)), m);
            double[][] db2 = MatrixOperations.divide(dZ2, m);

            //LAYER 1
            double[][] dZ1 = MatrixOperations.multiply(MatrixOperations.dot(MatrixOperations.T(W2), dZ2), MatrixOperations.subtract(1.0, MatrixOperations.power(A1, 2)));
            double[][] dW1 = MatrixOperations.divide(MatrixOperations.dot(dZ1, MatrixOperations.T(X)), m);
            double[][] db1 = MatrixOperations.divide(dZ1, m);

            // G.D
            W1 = MatrixOperations.subtract(W1, MatrixOperations.multiply(0.01, dW1));
            b1 = MatrixOperations.subtract(b1, MatrixOperations.multiply(0.01, db1));

            W2 = MatrixOperations.subtract(W2, MatrixOperations.multiply(0.01, dW2));
            b2 = MatrixOperations.subtract(b2, MatrixOperations.multiply(0.01, db2));

            if (i % 100 == 0) {
                MatrixOperations.print("==============");
                MatrixOperations.print("Cost = " + cost);
                MatrixOperations.print("Predictions = " + Arrays.deepToString(A2));
            }
        }
    }

    public double[][] classify(double[][] X){
        if(W1 != null && b1 != null && W2 != null && b2 != null) {
            double[][] Z1 = MatrixOperations.add(MatrixOperations.dot(W1, X), b1);
            double[][] A1 = MatrixOperations.sigmoid(Z1);

            double[][] Z2 = MatrixOperations.add(MatrixOperations.dot(W2, A1), b2);
            return MatrixOperations.sigmoid(Z2);
        }
        return null;
    }
}
