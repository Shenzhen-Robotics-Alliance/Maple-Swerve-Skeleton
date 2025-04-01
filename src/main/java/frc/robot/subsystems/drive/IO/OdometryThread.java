package frc.robot.subsystems.drive.IO;

import static frc.robot.constants.DriveTrainConstants.ODOMETRY_CACHE_CAPACITY;
import static frc.robot.constants.DriveTrainConstants.SIMULATION_TICKS_IN_1_PERIOD;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.drive.OdometryThreadReal;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.littletonrobotics.junction.AutoLog;

public interface OdometryThread {
    class OdometryInput {
        private final DoubleSupplier supplier;
        private final Queue<Double> queue;

        public OdometryInput(DoubleSupplier signal) {
            this.supplier = signal;
            this.queue = new ArrayBlockingQueue<>(ODOMETRY_CACHE_CAPACITY);
        }

        public void cacheInputToQueue() {
            this.queue.offer(supplier.getAsDouble());
        }

        public void writeToDoubleInput(double[] input) {
            writeToDoubleInput(input, d -> d);
        }

        public void writeToDoubleInput(double[] input, Function<Double, Double> mapper) {
            for (int i = 0; i < input.length; i++)
                input[i] = mapper.apply(Objects.requireNonNullElse(queue.poll(), 0.0));
        }

        public <T> void writeToInput(T[] input, Function<Double, T> mapper) {
            for (int i = 0; i < input.length; i++)
                input[i] = mapper.apply(Objects.requireNonNullElse(queue.poll(), 0.0));
            queue.clear();
        }
    }

    List<OdometryInput> registeredInputs = new ArrayList<>();
    List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();

    static OdometryInput registerSignalSignal(BaseStatusSignal signal) {
        registeredStatusSignals.add(signal);
        return registerInput(signal::getValueAsDouble);
    }

    static OdometryInput registerInput(DoubleSupplier supplier) {
        final OdometryInput odometryInput = new OdometryInput(supplier);
        registeredInputs.add(odometryInput);
        return odometryInput;
    }

    static OdometryThread createInstance(SwerveDrive.DriveType type) {
        return switch (Robot.CURRENT_ROBOT_MODE) {
            case REAL -> new OdometryThreadReal(
                    type,
                    registeredInputs.toArray(new OdometryInput[0]),
                    registeredStatusSignals.toArray(new BaseStatusSignal[0]));
            case SIM -> new OdometryThreadSim();
            case REPLAY -> inputs -> {};
        };
    }

    @AutoLog
    class OdometryThreadInputs {
        public int odometryTicksCountInPreviousRobotPeriod = 0;
        public double[] measurementTimeStamps = new double[ODOMETRY_CACHE_CAPACITY];
    }

    void updateInputs(OdometryThreadInputs inputs);

    default void start() {}

    default void lockOdometry() {}

    default void unlockOdometry() {}

    final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.odometryTicksCountInPreviousRobotPeriod = SIMULATION_TICKS_IN_1_PERIOD;
            final double robotStartingTimeStamps = Timer.getTimestamp(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs / SIMULATION_TICKS_IN_1_PERIOD;
            for (int i = 0; i < SIMULATION_TICKS_IN_1_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
