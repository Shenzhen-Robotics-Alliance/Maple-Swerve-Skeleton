package frc.robot.HardwareIO.Helpers;

import frc.robot.HardwareIO.Abstractions.RawMotor;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class LoggedMotor {
    public final RawMotor rawMotor;
    private final String logPath;
    private final int portOnPowerDistributionPanel;

    public LoggedMotor(String name, RawMotor rawMotor) {
        this(name, rawMotor, -1);
    }

    public LoggedMotor(String name, RawMotor rawMotor, int portOnPowerDistributionPanel) {
        this.rawMotor = rawMotor;
        this.logPath = "/MotorOutputs/" + name + "/";
        this.portOnPowerDistributionPanel = portOnPowerDistributionPanel;
    }


    public void setMotorPower(double power) {
        Logger.recordOutput(logPath + "relaxed", false);
        Logger.recordOutput(logPath + "locked", false);
        rawMotor.setOutPut(power);
        Logger.recordOutput(logPath + "requestedPower", power);
        if (0 <= portOnPowerDistributionPanel && portOnPowerDistributionPanel <= 15)
            Logger.recordOutput(logPath + "measuredPDPCurrent", RobotContainer.powerDistribution.getCurrent(portOnPowerDistributionPanel));
    }

    public void relax() {
        Logger.recordOutput(logPath + "relaxed", true);
        Logger.recordOutput(logPath + "locked", false);
        rawMotor.configureZeroPowerBehavior(RawMotor.ZeroPowerBehavior.RELAX);
        rawMotor.setOutPut(0);
    }

    public void lock() {
        Logger.recordOutput(logPath + "relaxed", false);
        Logger.recordOutput(logPath + "locked", true);
        rawMotor.configureZeroPowerBehavior(RawMotor.ZeroPowerBehavior.BREAK);
        rawMotor.setOutPut(0);
    }
}
