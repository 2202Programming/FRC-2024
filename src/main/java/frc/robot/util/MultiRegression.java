// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Data fitting tool for multi-independant + polynomial uses gradient descent
*/
public class MultiRegression {
    private double[] param;
    private int numFeatures;
    private int degree;
    /**
     * MultiRegression tool for fitting data with multi-independant- singleOutput
     * @param ind_val      independent variable values
     * @param dep_val      dependent variable values
     * @param numIteration number of iterations
     * @param learningRate learning rate
     * @param degree       degree of polynomial
     * @throws IllegalArgumentException ind_val and dep_val array lengths are not equal
     */
    public MultiRegression(double[][] ind_val, double[] dep_val, int numIteration, double learningRate, int degree) {
        if (ind_val.length != dep_val.length) {
            throw new IllegalArgumentException("array lengths are not equal");
        }
        numFeatures = ind_val[0].length;
        this.degree = degree;
        param = new double[(int)Math.pow(degree + 1, numFeatures)];
        train(ind_val, dep_val,learningRate, numIteration);
    }
    
    private void train(double[][] ind_val, double[] dep_val, double learningRate, int numIterations) {
        int m = dep_val.length; // sample number
        int n = param.length; // param number
        double[] errors = new double[m];

        // gradient descent
        for (int iter = 0; iter < numIterations; iter++) {
            for (int i = 0; i < m; i++) {
                double prediction = predict(ind_val[i]);
                errors[i] = prediction - dep_val[i];
            }
            for (int j = 0; j < n; j++) {
                double sumErrors = 0;
                for (int i = 0; i < m; i++) {
                    double[] featurePowers = calculateFeaturePowers(ind_val[i], j);
                    sumErrors += errors[i] * featurePowers[j];
                }
                param[j] -= learningRate * (1.0 / m) * sumErrors;
            }
        }
    }

    public double predict(double[] ind_val) {
        double prediction = 0;
        int index = 0;
        for (int i = 0; i <= degree; i++) {
            for (int j = 0; j < numFeatures; j++) {
                prediction += param[index++] * Math.pow(ind_val[j], i);
            }
        }
        return prediction;
    }

    private double[] calculateFeaturePowers(double[] features, int index) {
        double[] featurePowers = new double[numFeatures];
        int remainder = index;
        for (int i = numFeatures - 1; i >= 0; i--) {
            int power = remainder % (degree + 1);
            remainder /= (degree + 1);
            featurePowers[i] = Math.pow(features[i], power);
        }
        return featurePowers;
    }
}