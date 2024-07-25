package frc.robot.utils.MapleMaths;

import frc.robot.BuildConstants;

import java.util.Random;

public class MapleCommonMath {
    /**
     * random object that generates random variables
     * the seed is the hash of GIT_SHA
     * this way when you do log-replay even the generated random numbers are the same
     * */
    private static final Random random = new Random(BuildConstants.GIT_SHA.hashCode());
    public static double linearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x) {
        final double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        return linearInterpretation(x1, y1, x2, y2, Math.min(maxX, Math.max(minX, x)));
    }
    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    /**
     * using the random number generator of a fixed seed, generate the next random normal variable
     * @param mean the center of the distribution
     * @param stdDev the standard deviation of the distribution
     * @return the next random variable x from the distribution
     * */
    public static double generateRandomNormal(double mean, double stdDev) {
        double u1 = random.nextDouble();
        double u2 = random.nextDouble();
        // Box–Muller transform https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        double z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2);
        return z0 * stdDev + mean;
    }
}
