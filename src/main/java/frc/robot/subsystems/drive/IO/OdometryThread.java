package frc.robot.subsystems.drive.IO;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.OdometryThreadReal;
import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public interface OdometryThread {
    final class OdometryDoubleInput {
        private final Supplier<Double> supplier;
        private final Queue<Double> queue;

        public OdometryDoubleInput(Supplier<Double> signal) {
            this.supplier = signal;
            this.queue = new ArrayBlockingQueue<>(Constants.SwerveDriveConfigs.ODOMETRY_CACHE_CAPACITY);
        }

        public void cacheInputToQueue() {
            this.queue.offer(supplier.get());
        }
    }

    List<OdometryDoubleInput> registeredInputs = new ArrayList<>();
    List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();
    static Queue<Double> registerSignalInput(StatusSignal<Double> signal) {
        signal.setUpdateFrequency(Constants.SwerveDriveConfigs.ODOMETRY_FREQUENCY, Constants.SwerveDriveConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS);
        registeredStatusSignals.add(signal);
        return registerInput(signal.asSupplier());
    }
    static Queue<Double> registerInput(Supplier<Double> supplier) {
        final OdometryDoubleInput odometryDoubleInput = new OdometryDoubleInput(supplier);
        registeredInputs.add(odometryDoubleInput);
        return odometryDoubleInput.queue;
    }

    static OdometryThread createInstance() {
        if (Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REAL)
            return new OdometryThreadReal(
                    registeredInputs.toArray(new OdometryDoubleInput[0]),
                    registeredStatusSignals.toArray(new BaseStatusSignal[0])
            );
        return inputs -> {};
    }

    @AutoLog
    class OdometryThreadInputs {
        public double[] measurementTimeStamps = new double[0];
    }

    void updateInputs(OdometryThreadInputs inputs);

    default void start() {}

    default void lockOdometry() {}

    default void unlockOdometry() {}
}
