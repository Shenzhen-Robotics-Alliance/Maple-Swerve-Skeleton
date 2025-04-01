package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.Timer;

public class CanBusIOReal implements CanBusIO {
    private final Timer previousRefreshTime;
    private final CANBus canBus;

    public CanBusIOReal(CANBus canBus) {
        this.canBus = canBus;
        previousRefreshTime = new Timer();
        previousRefreshTime.start();
    }

    @Override
    public void updateInputs(CanBusInputs inputs) {
        if (!previousRefreshTime.hasElapsed(0.49)) return;
        CANBus.CANBusStatus status = canBus.getStatus();
        inputs.utilization = status.BusUtilization;
        inputs.offCount = status.BusOffCount;
        inputs.txFullCount = status.TxFullCount;
        inputs.receiveErrorCounter = status.REC;
        inputs.transmitErrorCounter = status.TEC;
        previousRefreshTime.reset();
    }
}
