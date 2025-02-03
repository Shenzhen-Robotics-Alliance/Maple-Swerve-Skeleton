package frc.robot.subsystems.drive.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface CanBusIO {
    class CanBusInputs implements LoggableInputs {
        public String name = "None";
        public double utilization = 0.0;
        public int offCount = 0;
        public int txFullCount = 0;
        public int receiveErrorCounter = 0;
        public int transmitErrorCounter = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("name", name);
            table.put("utilization", utilization);
            table.put("offCount", offCount);
            table.put("txFullCount", txFullCount);
            table.put("receiveErrorCounter", receiveErrorCounter);
            table.put("transmitErrorCounter", transmitErrorCounter);
        }

        @Override
        public void fromLog(LogTable table) {
            this.name = table.get("name", "None");
            this.utilization = table.get("utilization", 0.0);
            this.offCount = table.get("offCount", 0);
            this.txFullCount = table.get("txFullCount", 0);
            this.receiveErrorCounter = table.get("receiveErrorCounter", 0);
            this.transmitErrorCounter = table.get("transmitErrorCounter", 0);
        }
    }

    void updateInputs(CanBusInputs inputs);
}
