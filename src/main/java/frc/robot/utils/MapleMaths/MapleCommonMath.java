package frc.robot.utils.MapleMaths;

import frc.robot.BuildConstants;

import java.util.Random;

public class MapleCommonMath {
    public static double linearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x) {
        final double minX = Math.min(x1, x2), maxX = Math.max(x1, x2);
        return linearInterpretation(x1, y1, x2, y2, Math.min(maxX, Math.max(minX, x)));
    }
    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

}
