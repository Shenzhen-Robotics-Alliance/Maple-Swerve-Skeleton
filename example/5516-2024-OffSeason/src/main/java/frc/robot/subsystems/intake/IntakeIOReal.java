package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOReal implements IntakeIO {
    private final TalonFX intakeFalcon;
    private final DigitalInput lowerBeamBreaker, upperBeamBreaker;
    private final StatusSignal<Double> intakeCurrent;
    public IntakeIOReal(int intakeTalonFXID, int lowerBeamBreakerChannel, int upperBeamBreakerChannel) {
        this.intakeFalcon = new TalonFX(intakeTalonFXID);

        final CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.SupplyCurrentLimitEnable = true;
        currentLimitsConfigs.SupplyCurrentLimit = 40;
        this.intakeFalcon.getConfigurator().apply(currentLimitsConfigs);
        this.intakeCurrent = intakeFalcon.getSupplyCurrent();
        this.lowerBeamBreaker = new DigitalInput(lowerBeamBreakerChannel);
        this.upperBeamBreaker = new DigitalInput(upperBeamBreakerChannel);
        BaseStatusSignal.setUpdateFrequencyForAll(50, intakeCurrent);
        intakeFalcon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        BaseStatusSignal.refreshAll(intakeCurrent);
        inputs.motorCurrent = intakeCurrent.getValue();
        inputs.lowerBeamBreakBlocked = !lowerBeamBreaker.get();
        inputs.upperBeamBreakerBlocked = !upperBeamBreaker.get();
    }

    @Override
    public void runIntakeVoltage(double volts) {
        intakeFalcon.setControl(new VoltageOut(volts).withEnableFOC(true));
    }
}
