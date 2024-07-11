package frc.robot.utils.MechanismControl;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.MapleInterpolationTable;

import java.io.IOException;

public class InterpolatedMotorFeedForward extends SimpleMotorFeedforward {
    private final String name;
    private final MapleInterpolationTable interpolationTable;
    public InterpolatedMotorFeedForward(String name, double[] motorPower, double[] velocityMeasured) {
        this(new MapleInterpolationTable(
                name,
                new MapleInterpolationTable.Variable("motorVelocity", velocityMeasured),
                new MapleInterpolationTable.Variable("motorPower", motorPower)
        ));
    }

    protected InterpolatedMotorFeedForward(MapleInterpolationTable interpolationTable) {
        super(0, 0);
        this.name = interpolationTable.tableName;
        this.interpolationTable = interpolationTable;
    }

    @Override
    public double calculate(double velocity) {
        if (velocity == 0)
            return 0;
        final double motorPowerMagnitude = interpolationTable.interpolateVariable(
                "motorPower",
                Math.abs(velocity)
        );
        return Math.copySign(motorPowerMagnitude, velocity);
    }

    public void saveFeedForwardConfigToUSB() {
        final MapleConfigFile configFile = interpolationTable.toConfigFile("InterpolatedMotorFeedForward");
        configFile.saveConfigToUSBSafe();
    }

    public static InterpolatedMotorFeedForward fromDeployedDirectory(String name) {
        final MapleInterpolationTable interpolationTable;
        try {
            interpolationTable = MapleInterpolationTable.fromConfigFile(MapleConfigFile.fromDeployedConfig(
                    "InterpolatedMotorFeedForward", name
            ));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        return new InterpolatedMotorFeedForward(interpolationTable);
    }
}
