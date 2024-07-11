package frc.robot.utils.MechanismControl;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.Config.MapleInterpolationTable;
import org.littletonrobotics.junction.Logger;

public class InterpolatedMotorFeedForward extends SimpleMotorFeedforward {
    private final String name;
    private final MapleInterpolationTable interpolationTable;
    public InterpolatedMotorFeedForward(String name, double[] motorPower, double[] velocityMeasured) {
        super(0, 0);
        this.name = name;
        this.interpolationTable = new MapleInterpolationTable(
                name,
                new MapleInterpolationTable.Variable("motorVelocity", velocityMeasured),
                new MapleInterpolationTable.Variable("motorPower", motorPower)
        );
    }

    @Override
    public double calculate(double velocity) {
        if (velocity == 0)
            return 0;
        final double motorPowerMagnitude = interpolationTable.interpolateVariable(
                "motorPower",
                Math.abs(velocity)
        );
        Logger.recordOutput("/MotorFeedForward/" + name + "/required velocity", velocity);
        Logger.recordOutput("/MotorFeedForward/" + name + "/corresponding power (mag)", motorPowerMagnitude);
        return Math.copySign(motorPowerMagnitude, velocity);
    }

    public void saveFeedForwardConfigToUSB() {
        final MapleConfigFile configFile = interpolationTable.toConfigFile("InterpolatedMotorFeedForward");
        configFile.saveConfigToUSBSafe();
    }
}
