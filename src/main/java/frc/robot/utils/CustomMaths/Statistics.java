package frc.robot.utils.CustomMaths;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

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
    /**
     *
     *
     * <h2>Stores a numerical estimation of something known to be coming from a normal distribution.</h2>
     *
     * @param center the center of the estimation
     * @param standardDeviation the standard deviation of the estimation
     */
    public record Estimation(double center, double standardDeviation) {}

    /**
     *
     *
     * <h2>Apply a linear filter to find the best estimation over a list of estimations.</h2>
     *
     * @see #linearFilter(Estimation...)
     */
    public static Estimation linearFilter(List<Estimation> estimations) {
        return linearFilter(estimations.toArray(new Estimation[0]));
    }

    /**
     *
     *
     * <h2>Apply a linear filter to find the best estimation over a few estimations.</h2>
     *
     * <p>The estimations with less standard deviation get trusted more.
     *
     * @return the overall best estimation according to all the given estimations
     */
    public static Estimation linearFilter(Estimation... estimations) {
        if (estimations == null || estimations.length == 0)
            throw new IllegalArgumentException("At least one estimation is required.");

        if (estimations.length == 1) return estimations[0];

        double sumWeightedCenters = 0.0;
        double sumWeights = 0.0;

        for (Estimation estimation : estimations) {
            double variance = estimation.standardDeviation() * estimation.standardDeviation();
            double weight = 1.0 / variance;
            sumWeightedCenters += weight * estimation.center();
            sumWeights += weight;
        }

        double combinedCenter = sumWeightedCenters / sumWeights;
        double combinedStandardDeviation = Math.sqrt(1.0 / sumWeights);

        return new Estimation(combinedCenter, combinedStandardDeviation);
    }

    /**
     *
     *
     * <h2>Stores a rotational estimation of something known to be coming from a normal distribution.</h2>
     *
     * @param center the center of the estimation
     * @param standardDeviationRad the standard deviation of the estimation, in radians
     */
    public record RotationEstimation(Rotation2d center, double standardDeviationRad) {}

    /**
     *
     *
     * <h2>Apply a rotational filter to find the best estimation over a list of rotational estimations.</h2>
     *
     * @see #rotationFilter(RotationEstimation...)
     * @return the overall best rotational estimation according to all the given estimations
     */
    public static RotationEstimation rotationFilter(List<RotationEstimation> rotationEstimations) {
        return rotationFilter(rotationEstimations.toArray(new RotationEstimation[0]));
    }

    /**
     *
     *
     * <h2>Apply a rotational filter to find the best estimation over a few rotational estimations.</h2>
     *
     * <p>The estimations with less standard deviation get trusted more.
     *
     * @return the overall best rotational estimation according to all the given estimations
     */
    public static RotationEstimation rotationFilter(RotationEstimation... rotationEstimations) {
        if (rotationEstimations == null || rotationEstimations.length == 0)
            throw new IllegalArgumentException("At least one rotation estimation is required.");

        if (rotationEstimations.length == 1) return rotationEstimations[0];

        // Transform the rotation estimations into Estimation objects for cosines
        List<Estimation> cosEstimations = Arrays.stream(rotationEstimations)
                .map(est -> new Estimation(Math.cos(est.center().getRadians()), est.standardDeviationRad()))
                .collect(Collectors.toList());

        // Apply linearFilter to compute the weighted mean of cosines
        Estimation cosEstimation = linearFilter(cosEstimations);

        // Transform the rotation estimations into Estimation objects for sines
        List<Estimation> sinEstimations = Arrays.stream(rotationEstimations)
                .map(est -> new Estimation(Math.sin(est.center().getRadians()), est.standardDeviationRad()))
                .collect(Collectors.toList());

        // Apply linearFilter to compute the weighted mean of sines
        Estimation sinEstimation = linearFilter(sinEstimations);

        // Compute the resultant length R
        double R = Math.sqrt(Math.pow(cosEstimation.center(), 2) + Math.pow(sinEstimation.center(), 2));

        // Compute the circular standard deviation
        double circularStdDev;
        if (R > 0) circularStdDev = Math.sqrt(-2.0 * Math.log(R));
        else
            // If R is 0, the angles are uniformly distributed; standard deviation is maximal
            circularStdDev = Math.PI;

        // Compute the mean angle using the weighted means of cosines and sines
        double meanTheta = Math.atan2(sinEstimation.center(), cosEstimation.center());

        // Normalize the angle to be within [-pi, pi]
        meanTheta = Rotation2d.fromRadians(meanTheta).getRadians();

        return new RotationEstimation(new Rotation2d(meanTheta), circularStdDev);
    }
}
