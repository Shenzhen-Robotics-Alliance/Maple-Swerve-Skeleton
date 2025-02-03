package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.CANBus;

public class CanBusIOReal implements CanBusIO {
    private final CANBus canBus;

    public CanBusIOReal(CANBus canBus) {
        this.canBus = canBus;
    }

    @Override
    public void updateInputs(CanBusInputs inputs) {
        CANBus.CANBusStatus status = canBus.getStatus();
        inputs.utilization = status.BusUtilization;
        inputs.offCount = status.BusOffCount;
        inputs.txFullCount = status.TxFullCount;
        inputs.receiveErrorCounter = status.REC;
        inputs.transmitErrorCounter = status.TEC;
    }
}
