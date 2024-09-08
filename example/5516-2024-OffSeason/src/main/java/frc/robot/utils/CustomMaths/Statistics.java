package frc.robot.utils.CustomMaths;

public final class Statistics {
    public static double getMean(double[] dataSet) {
        double sum = 0;
        for (double data : dataSet) sum += data;
        return sum / dataSet.length;
    }

    public static double getMedian(double[] dataSet) {
        if (dataSet.length % 2 == 0)
            return (dataSet[dataSet.length / 2 - 1] + dataSet[dataSet.length / 2]) / 2;

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
        for (int i = 0; i < dataSet.length; i++)
            standardizedScores[i] = (dataSet[i] - mean) / standardDeviation;
        return standardizedScores;
    }

    public static double getCorrelationCoefficient(double[] dataSet1, double[] dataSet2) {
        if (dataSet1.length != dataSet2.length)
            throw new IllegalArgumentException("data set length unmatched");
        final double[] standardizedScores1 = getStandardizedScores(dataSet1),
                standardizedScores2 = getStandardizedScores(dataSet2);
        double productSum = 0;
        for (int i = 0; i < dataSet2.length; i++)
            productSum += standardizedScores1[i] * standardizedScores2[i];
        return productSum / (dataSet1.length - 1);
    }

    public static double getBestFitLineSlope(double[] dataSetX, double[] dataSetY) {
        final double standardizedDeviationX = getStandardDeviation(dataSetX),
                standardizedDeviationY = getStandardDeviation(dataSetY);
        return getCorrelationCoefficient(dataSetX, dataSetY)
                * standardizedDeviationY
                / standardizedDeviationX;
    }

    public static double getBestFitLineIntersect(double[] dataSetX, double[] dataSetY) {
        final double slope = getBestFitLineSlope(dataSetX, dataSetY);
        return getMean(dataSetY) - slope * getMean(dataSetX);
    }
}
