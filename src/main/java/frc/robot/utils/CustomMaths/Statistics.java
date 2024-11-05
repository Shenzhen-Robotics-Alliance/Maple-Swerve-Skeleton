package frc.robot.utils.CustomMaths;

import java.util.Arrays;
import java.util.List;

public final class Statistics {
    public static double getMean(double[] dataSet) {
        double sum = 0;
        for (double data : dataSet) sum += data;
        return sum / dataSet.length;
    }

    public static double getMedian(double[] dataSet) {
        if (dataSet.length % 2 == 0) return (dataSet[dataSet.length / 2 - 1] + dataSet[dataSet.length / 2]) / 2;

        return dataSet[dataSet.length / 2];
    }

    public static double getStandardDeviation(double[] dataSet) {
        final double mean = getMean(dataSet);
        double varianceSum = 0;
        for (double data : dataSet) varianceSum += (data - mean) * (data - mean);
        return Math.sqrt(varianceSum / (dataSet.length - 1));
    }

    public static double[] getStandardizedScores(double[] dataSet) {
        if (dataSet.length <= 1) throw new IllegalArgumentException("data too short!!!");
        final double[] standardizedScores = new double[dataSet.length];
        final double mean = getMean(dataSet);
        final double standardDeviation = getStandardDeviation(dataSet);
        for (int i = 0; i < dataSet.length; i++) standardizedScores[i] = (dataSet[i] - mean) / standardDeviation;
        return standardizedScores;
    }

    public static double getCorrelationCoefficient(double[] dataSet1, double[] dataSet2) {
        if (dataSet1.length != dataSet2.length) throw new IllegalArgumentException("data set length unmatched");
        final double[] standardizedScores1 = getStandardizedScores(dataSet1),
                standardizedScores2 = getStandardizedScores(dataSet2);
        double productSum = 0;
        for (int i = 0; i < dataSet2.length; i++) productSum += standardizedScores1[i] * standardizedScores2[i];
        return productSum / (dataSet1.length - 1);
    }

    public static double getBestFitLineSlope(double[] dataSetX, double[] dataSetY) {
        final double standardizedDeviationX = getStandardDeviation(dataSetX),
                standardizedDeviationY = getStandardDeviation(dataSetY);
        return getCorrelationCoefficient(dataSetX, dataSetY) * standardizedDeviationY / standardizedDeviationX;
    }

    public static double getBestFitLineIntersect(double[] dataSetX, double[] dataSetY) {
        final double slope = getBestFitLineSlope(dataSetX, dataSetY);
        return getMean(dataSetY) - slope * getMean(dataSetX);
    }

    public record Estimation(double center, double standardDeviation) {}

    public static Estimation linearFilter(List<Estimation> estimations) {
        return linearFilter(estimations.toArray(Estimation[]::new));
    }

    /** Given a set of estimations towards a value, calculates the overall estimation */
    public static Estimation linearFilter(Estimation... estimations) {
        if (estimations == null || estimations.length == 0)
            throw new IllegalArgumentException("At least one estimation is required.");

        if (estimations.length == 1) return estimations[0];

        double sumWeightedCenters = 0.0;
        double sumWeights = 0.0;

        for (Estimation estimation : estimations) {
            double variance = estimation.standardDeviation * estimation.standardDeviation;
            double weight = 1.0 / variance;
            sumWeightedCenters += weight * estimation.center;
            sumWeights += weight;
        }

        double combinedCenter = sumWeightedCenters / sumWeights;
        double combinedStandardDeviation = Math.sqrt(1.0 / sumWeights);

        return new Estimation(combinedCenter, combinedStandardDeviation);
    }

    public static double getStandardDeviation(List<Estimation> estimations) {
        return getStandardDeviation(estimations.toArray(Estimation[]::new));
    }

    public static double getStandardDeviation(Estimation... estimations) {
        return getStandardDeviation(Arrays.stream(estimations)
                .mapToDouble(estimation -> estimation.center)
                .toArray());
    }
}
