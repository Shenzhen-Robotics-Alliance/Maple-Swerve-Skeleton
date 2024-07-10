// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.utils.Config;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class MapleInterpolationTable {
    private final String tableName;
    private final Variable independentVariable;
    private final Map<String, Variable> interpolatedVariables;

    public static final class Variable {
        private String tableName;
        public final String variableName;
        public final double[] values;

        public Variable(String name, double... values) {
            this.tableName = "Unknown";
            this.variableName = name;
            this.values = values;
        }

        public void initializeTuningPanelOnDashboard() {
//            if (Objects.equals(tableName, "Unknown"))
//                return;
//            SmartDashboard.putNumberArray("InterpolationTables/" + tableName + "/" + variableName, values);
        }

        public void updateValuesFromDashboard() {
            if (Objects.equals(tableName, "Unknown"))
                return;

            final double[] updatedValues = SmartDashboard.getNumberArray("InterpolationTables/" + tableName + "/" + variableName, values);
            if (updatedValues.length != values.length)
                return;

            System.arraycopy(updatedValues, 0, values, 0, values.length);
        }
    }
    public MapleInterpolationTable(String name, Variable independentVariable, Variable... interpolatedVariables) {
        this.tableName = name;
        this.independentVariable = independentVariable;
        this.interpolatedVariables = new HashMap<>();
        for (Variable variable:interpolatedVariables)
            this.interpolatedVariables.put(variable.variableName, variable);

        initDashboardTunings();
    }

    private void initDashboardTunings() {
        independentVariable.tableName = this.tableName;
        independentVariable.initializeTuningPanelOnDashboard();
        for (Variable interpolatedVariable: interpolatedVariables.values()) {
            interpolatedVariable.tableName = this.tableName;
            interpolatedVariable.initializeTuningPanelOnDashboard();
        }
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

    public double interpolateVariable(String interpolatedVariableName, double independentVariableValue) {
        if (!interpolatedVariables.containsKey(interpolatedVariableName))
            throw new NullPointerException("interpolated variable does not exit: " + interpolatedVariableName);
        final Variable interpolatedVariable = interpolatedVariables.get(interpolatedVariableName);

        independentVariable.updateValuesFromDashboard();
        interpolatedVariable.updateValuesFromDashboard();

        final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap =
                getInterpolatingDoubleTreeMap(independentVariable, interpolatedVariables.get(interpolatedVariableName));

        return interpolatingDoubleTreeMap.get(independentVariableValue);
    }

    public MapleConfigFile toConfigFile(String configType) {
        final MapleConfigFile configFile = new MapleConfigFile(configType, tableName);
        // TODO write this part
        return configFile;
    }
}
