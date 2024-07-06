package frc.robot.utils.Math;

public class LookUpTable {
    public final double[] xValues;
    public final double[] yValues;
    final int n;

    public LookUpTable(double[] xValues, double[] yValues) {
        if (xValues.length != yValues.length)
            throw new IllegalArgumentException("look up table length not match");
        double prev_x = -Double.POSITIVE_INFINITY;
        for (double x : xValues) {
            if (x < prev_x)
                throw new IllegalArgumentException("look up table X must be in increasing order");
            prev_x = x;
        }
        this.xValues = xValues;
        this.yValues = yValues;
        this.n = xValues.length;
    }

    public double getYPrediction(double x) {
        if (x < xValues[0]) return yValues[0];
        for (int i = 0; i < n - 1; i++)
            if (xValues[i] < x && x < xValues[i + 1])
                return linearInterpretation(xValues[i], yValues[i], xValues[i + 1], yValues[i + 1], x);
        return yValues[yValues.length - 1];
    }

    public static double linearInterpretationWithBounding(
            double x1, double y1, double x2, double y2, double x) {
        final double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        return linearInterpretation(x1, y1, x2, y2, Math.min(maxX, Math.max(minX, x)));
    }

    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }
}
