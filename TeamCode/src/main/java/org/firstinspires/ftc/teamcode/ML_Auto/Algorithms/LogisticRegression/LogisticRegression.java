package org.firstinspires.ftc.teamcode.ML_Auto.Algorithms.LogisticRegression;

import java.io.Serializable;
import java.util.Arrays;
import java.util.List;

public class LogisticRegression implements Serializable {
    private static final long serialVersionUID = 1L;

    public static final int DEFAULT_N = 3;
    public static final double DEFAULT_RATE = 0.0001;
    public static final int DEFAULT_ITERATIONS = 50000;

    /**
     * the learning rate
     */
    private double rate;

    /**
     * the weight to learn
     */
    private double[] weights;

    /**
     * the number of iterations
     */
    private int ITERATIONS;

    public LogisticRegression(int n, int iterations) {
        this.rate = 0.0001;
        weights = new double[n];
        ITERATIONS = iterations;
    }

    private static double sigmoid(double z) {
        return 1.0 / (1.0 + Math.exp(-z));
    }

    public void train(List<LogisticData> instances) {
        for (int n = 0; n < ITERATIONS; n++) {
            double lik = 0.0;
            for (int i = 0; i < instances.size(); i++) {
                int[] x = instances.get(i).x;
                double predicted = classify(x);
                int label = instances.get(i).label;
                for (int j = 0; j < weights.length; j++) {
                    weights[j] = weights[j] + rate * (label - predicted) * x[j];
                }
                // not necessary for learning
                lik += label * Math.log(classify(x)) + (1 - label) * Math.log(1 - classify(x));
            }
            System.out.println("iteration: " + n + " " + Arrays.toString(weights) + " mle: " + lik);
        }
    }

    private double classify(int[] x) {
        double logit = .0;
        for (int i = 0; i < weights.length; i++) {
            logit += weights[i] * x[i];
        }
        return sigmoid(logit);
    }

    /*public static List<LogisticData> readDataSet(String file) throws FileNotFoundException {
        List<LogisticData> dataset = new ArrayList<>();
        Scanner scanner = null;
        try {
            scanner = new Scanner(new File(file));
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                if (line.startsWith("#")) {
                    continue;
                }
                String[] columns = line.split("\\s+");

                // skip first column and last column is the label
                int i = 1;
                int[] data = new int[columns.length - 2];
                for (i = 1; i < columns.length - 1; i++) {
                    data[i - 1] = Integer.parseInt(columns[i]);
                }
                int label = Integer.parseInt(columns[i]);
                Instance instance = new Instance(label, data);
                dataset.add(instance);
            }
        } finally {
            if (scanner != null)
                scanner.close();
        }
        return dataset;
    }*/


    /*public static void main(String... args) throws FileNotFoundException {
        List<Instance> instances = readDataSet("dataset.txt");
        Logistic logistic = new Logistic(5);
        logistic.train(instances);
        int[] x = {2, 1, 1, 0, 1};
        System.out.println("prob(1|x) = " + logistic.classify(x));

        int[] x2 = {1, 0, 1, 0, 0};
        System.out.println("prob(1|x2) = " + logistic.classify(x2));

    }*/
}
