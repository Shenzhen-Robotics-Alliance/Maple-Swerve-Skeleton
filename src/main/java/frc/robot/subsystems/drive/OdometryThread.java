package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public interface OdometryThread {
    class OdometryThreadInputs implements LoggableInputs {
        public double[] measurementTimeStamps = new double[0];

        @Override
        public void toLog(LogTable table) {
            table.put("measurementTimeStamps", measurementTimeStamps);
        }

        @Override
        public void fromLog(LogTable table) {
            measurementTimeStamps = table.get("measurementTimeStamps", new double[0]);
        }
    }

    class OdometryDoubleInput {
        private final Supplier<Double> supplier;
        private final Queue<Double> queue;
        private double[] valuesSincePreviousPeriod = new double[0];
        private double latestValue;

        public OdometryDoubleInput(Supplier<Double> signal) {
            this.supplier = signal;
            this.latestValue = 0;
            this.queue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);
        }

        public double[] getValuesSincePreviousPeriod() {
            return valuesSincePreviousPeriod;
        }

        public void cacheInputToQueue() {
            queue.offer(supplier.get());
            latestValue = supplier.get();
        }

        public void fetchQueueToArray() {
            valuesSincePreviousPeriod = mapQueueToDoubleArray(queue);
            queue.clear();
        }

        public double getLatest() {
            return latestValue;
        }
    }

    List<OdometryDoubleInput> registeredInputs = new ArrayList<>();
    List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();
    static OdometryThread.OdometryDoubleInput registerSignalInput(StatusSignal<Double> signal) {
        signal.setUpdateFrequency(Constants.ChassisConfigs.ODOMETRY_FREQUENCY, Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS);
        registeredStatusSignals.add(signal);
        return registerInput(signal.asSupplier());
    }
    static OdometryThread.OdometryDoubleInput registerInput(Supplier<Double> supplier) {
        final OdometryThread.OdometryDoubleInput odometryDoubleInput = new OdometryThread.OdometryDoubleInput(supplier);
        registeredInputs.add(odometryDoubleInput);
        return odometryDoubleInput;
    }

    static double[] mapQueueToDoubleArray(Queue<Double> queue) {
        return queue.stream().mapToDouble(value -> value).toArray();
    }

    static OdometryThread createInstance() {
        if (Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REAL)
            return new OdometryThreadReal(
                    registeredInputs.toArray(new OdometryThread.OdometryDoubleInput[0]),
                    registeredStatusSignals.toArray(new BaseStatusSignal[0])
            );
        return new OdometryThread() {
            @Override public void updateInputs(OdometryThreadInputs inputs) {}
            @Override public void start() {}
        };

    }

    void updateInputs(OdometryThreadInputs inputs);

    void start();
}
