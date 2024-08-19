// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.utils.CustomConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class MapleInterpolationTable {
    public final String tableName;
    private final Variable independentVariable;
    private final Map<String, Variable> interpolatedVariables;
    public final double minX, maxX;

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
            if (Objects.equals(tableName, "Unknown"))
                return;
            SmartDashboard.putNumberArray("InterpolationTables/" + tableName + "/" + variableName, values);
        }

        public void updateValuesFromDashboard() {
            if (Objects.equals(tableName, "Unknown"))
                return;

            final double[] updatedValues = SmartDashboard.getNumberArray("InterpolationTables/" + tableName + "/" + variableName, new double[]{});
            if (updatedValues.length != values.length)
                return;
            System.arraycopy(updatedValues, 0, values, 0, values.length);
        }

        public void toBlock(MapleConfigFile.ConfigBlock block) {
            block.putStringConfig("name", variableName);
            for (int i = 0; i < values.length; i++)
                block.putDoubleConfig("sample"+i, values[i]);
        }

        public static Variable fromBlock(MapleConfigFile.ConfigBlock block, int length) {
            final String name = block.getStringConfig("name");
            final double[] values = new double[length];
            for (int i = 0; i < length; i++)
                values[i] = block.getDoubleConfig("sample"+i);

            return new Variable(name, values);
        }
    }
    public MapleInterpolationTable(String name, Variable independentVariable, Variable... interpolatedVariables) {
        this.tableName = name;
        this.independentVariable = independentVariable;
        this.interpolatedVariables = new HashMap<>();
        for (Variable variable:interpolatedVariables) {
            this.interpolatedVariables.put(variable.variableName, variable);
            if (variable.values.length != independentVariable.values.length)
                throw new RuntimeException("interpolated variable " + variable.variableName + " has length " + variable.values.length + " which does not match the independent variable");
        }

        this.minX = Arrays.stream(independentVariable.values).min().orElse(0);
        this.maxX = Arrays.stream(independentVariable.values).max().orElse(0);

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
        targetMap.clear();
        for (int i = 0; i < independentVariable.values.length; i++)
            targetMap.put(independentVariable.values[i], interpolatedVariable.values[i]);
        return targetMap;
    }

    public double interpolateVariableWithLimit(String interpolatedVariableName, double independentVariableValue) {
        return interpolateVariable(interpolatedVariableName, MathUtil.clamp(independentVariableValue, minX, maxX));
    }

    public double interpolateVariable(String interpolatedVariableName, double independentVariableValue) {
        if (!interpolatedVariables.containsKey(interpolatedVariableName))
            throw new NullPointerException("interpolated variable does not exit: " + interpolatedVariableName);
        final Variable interpolatedVariable = interpolatedVariables.get(interpolatedVariableName);

        independentVariable.updateValuesFromDashboard();
        interpolatedVariable.updateValuesFromDashboard();

        final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap = getInterpolatingDoubleTreeMap(
                independentVariable, interpolatedVariable
        );

        return interpolatingDoubleTreeMap.get(independentVariableValue);
    }

    public double findDerivative(String interpolatedVariableName, double independentVariableValue, double dx) {
        final double dy =
                interpolateVariable(interpolatedVariableName, independentVariableValue + dx/2)
                        - interpolateVariable(interpolatedVariableName, independentVariableValue - dx/2);

        return dy / dx;
    }

    public MapleConfigFile toConfigFile(String configType) {
        final MapleConfigFile configFile = new MapleConfigFile(configType, tableName);

        final MapleConfigFile.ConfigBlock infoBlock = configFile.getBlock("info");
        infoBlock.putIntConfig("interpolatedVariablesCount", interpolatedVariables.size());
        infoBlock.putIntConfig("length", independentVariable.values.length);
        independentVariable.toBlock(configFile.getBlock("variable0"));
        int i = 1;
        for (Variable interpolatedVariable:interpolatedVariables.values())
            interpolatedVariable.toBlock(configFile.getBlock("variable" + (i++)));

        return configFile;
    }

    public static MapleInterpolationTable fromConfigFile(MapleConfigFile configFile) {
        final MapleConfigFile.ConfigBlock infoBlock = configFile.getBlock("info");
        final int interpolatedVariablesCount = infoBlock.getIntConfig("interpolatedVariablesCount"),
                length = infoBlock.getIntConfig("length");

        final Variable independentVariable = Variable.fromBlock(configFile.getBlock("variable0"), length);
        final Variable[] interpolateVariables = new Variable[interpolatedVariablesCount];
        for (int i = 0; i < interpolatedVariablesCount; i++)
            interpolateVariables[i] = Variable.fromBlock(configFile.getBlock("variable"+(i+1)), length);
        return new MapleInterpolationTable(configFile.configName, independentVariable, interpolateVariables); // TODO: write this part
    }
}
