package frc.robot.utils.Config;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import java.util.HashMap;
import java.util.Map;

public class MapleInterpolationTable {
    // TODO store to and load from maple config file
    private final String name;
    private final Variable independentVariable;
    private final Variable[] interpolatedVariables;
    private final Map<String, InterpolatingDoubleTreeMap> interpolatedVariableToInterpolationMapsMap;

    public static final class Variable {
        private final String tableName;
        public final String variableName;
        public final double[] values;

        public Variable(String tableName, String name, double[] values) {
            this.tableName = tableName;
            this.variableName = name;
            this.values = values;
        }
    }
    public MapleInterpolationTable(String name, Variable independentVariable, Variable... interpolatedVariables) {
        this.name = name;
        this.independentVariable = independentVariable;
        this.interpolatedVariables = interpolatedVariables;
        this.interpolatedVariableToInterpolationMapsMap = new HashMap<>();

        for (Variable interpolatedVariable:interpolatedVariables)
            interpolatedVariableToInterpolationMapsMap.put(
                    interpolatedVariable.variableName,
                    getInterpolatingDoubleTreeMap(independentVariable, interpolatedVariable)
            );
    }

    private static InterpolatingDoubleTreeMap getInterpolatingDoubleTreeMap(Variable independentVariable, Variable interpolatedVariable) {
        return getInterpolatingDoubleTreeMap(independentVariable, interpolatedVariable, new InterpolatingDoubleTreeMap());
    }
    private static InterpolatingDoubleTreeMap getInterpolatingDoubleTreeMap(Variable independentVariable, Variable interpolatedVariable, InterpolatingDoubleTreeMap targetMap) {
        if (interpolatedVariable.values.length != independentVariable.values.length)
            throw new IllegalArgumentException("interpolated variable " + interpolatedVariable.variableName + "does match the independent variable " + independentVariable.variableName + " in length");
        targetMap.clear();
        for (int i = 0; i < independentVariable.values.length; i++)
            targetMap.put(independentVariable.values[i], interpolatedVariable.values[i]);
        return targetMap;
    }

    public double interpolateVariable(String interpolatedVariableName, double independentVariable) {
        if (!interpolatedVariableToInterpolationMapsMap.containsKey(interpolatedVariableName))
            throw new NullPointerException("interpolated variable does not exit: " + interpolatedVariableName);

        final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = interpolatedVariableToInterpolationMapsMap.get(interpolatedVariableName);
        return interpolatingDoubleTreeMap.get(independentVariable);
    }
}
