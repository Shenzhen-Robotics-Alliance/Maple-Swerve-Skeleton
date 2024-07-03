package frc.robot.utils;

import java.util.List;

public class ArrayUtils {
    public static double[] toDoubleArray(List<Double> originalList) {
        double[] array = new double[originalList.size()];
        for (int i = 0; i < originalList.size(); i++) array[i] = originalList.get(i);
        return array;
    }

    public static double[] toDoubleArray(Double[] originalArray) {
        double[] array = new double[originalArray.length];
        for (int i = 0; i < originalArray.length; i++) array[i] = originalArray[i];
        return array;
    }

    public static void toDoubleList(double[] data, List<Double> targetList) {
        targetList.clear();
        for (double d : data) targetList.add(d);
    }
}
