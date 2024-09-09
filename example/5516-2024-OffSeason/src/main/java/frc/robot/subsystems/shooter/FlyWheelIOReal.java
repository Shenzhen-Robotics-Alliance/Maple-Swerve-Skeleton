package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlyWheelIOReal implements FlyWheelIO {
    private final TalonFX flyWheelFalcon;
    private final StatusSignal<Double> positionRevolutions, velocityRevolutions, supplyCurrentAmps;

    public FlyWheelIOReal(int talonFXID, boolean inverted) {
        flyWheelFalcon = new TalonFX(talonFXID);
        flyWheelFalcon.setInverted(inverted);

        this.positionRevolutions = flyWheelFalcon.getPosition();
        this.velocityRevolutions = flyWheelFalcon.getVelocity();
        this.supplyCurrentAmps = flyWheelFalcon.getSupplyCurrent();

        final CurrentLimitsConfigs currentLimitConfiguration = new CurrentLimitsConfigs();
        currentLimitConfiguration.SupplyCurrentLimitEnable = true;
        currentLimitConfiguration.SupplyCurrentLimit = 60;
        flyWheelFalcon.getConfigurator().apply(currentLimitConfiguration);
        BaseStatusSignal.setUpdateFrequencyForAll(100,
                positionRevolutions, velocityRevolutions, supplyCurrentAmps);
        flyWheelFalcon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlyWheelsInputs inputs) {
        BaseStatusSignal.refreshAll(positionRevolutions, velocityRevolutions, supplyCurrentAmps);

        inputs.flyWheelPositionRevs = positionRevolutions.getValue();
        inputs.flyWheelVelocityRevsPerSec = velocityRevolutions.getValue();
        inputs.supplyCurrentAmps = supplyCurrentAmps.getValue();
    }

    @Override
    public void runVoltage(double volts) {
        VoltageOut voltageOut = new VoltageOut(volts).withEnableFOC(false);
        this.flyWheelFalcon.setControl(voltageOut);
    }
}
