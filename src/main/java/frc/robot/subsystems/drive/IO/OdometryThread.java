package frc.robot.subsystems.drive.IO;

import static frc.robot.subsystems.drive.DriveTrainConfigs.*;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.utils.MapleTimeUtils;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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

    @AutoLog
    class OdometryThreadInputs {
        public int odometryTicksCountInPreviousRobotPeriod = 0;
        public double[] measurementTimeStamps = new double[ODOMETRY_CACHE_CAPACITY];
    }

    void updateInputs(OdometryThreadInputs inputs);

    default void start() {}

    default void lockOdometry() {}

    default void unlockOdometry() {}

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.odometryTicksCountInPreviousRobotPeriod = SIMULATION_TICKS_IN_1_PERIOD;
            final double robotStartingTimeStamps = Timer.getTimestamp(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs / SIMULATION_TICKS_IN_1_PERIOD;
            for (int i = 0; i < SIMULATION_TICKS_IN_1_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }

    public static final class OdometryThreadReal extends Thread implements OdometryThread {
        private final OdometryInput[] odometryDoubleInputs;
        private final BaseStatusSignal[] statusSignals;
        private final Queue<Double> timeStampsQueue;
        private final Lock lock = new ReentrantLock();

        public OdometryThreadReal() {
            this.timeStampsQueue = new ArrayBlockingQueue<>(ODOMETRY_CACHE_CAPACITY);
            this.odometryDoubleInputs = OdometryThread.registeredInputs.toArray(OdometryInput[]::new);
            this.statusSignals = OdometryThread.registeredStatusSignals.toArray(BaseStatusSignal[]::new);

            setName("OdometryThread");
            setDaemon(true);
        }

        @Override
        public synchronized void start() {
            if (odometryDoubleInputs.length > 0) super.start();
        }

        @Override
        public void run() {
            while (true) odometryPeriodic();
        }

        private void odometryPeriodic() {
            refreshSignalsAndBlockThread();

            lock.lock();
            timeStampsQueue.offer(estimateAverageTimeStamps());
            for (OdometryInput odometryInput : odometryDoubleInputs) odometryInput.cacheInputToQueue();
            lock.unlock();
        }

        private void refreshSignalsAndBlockThread() {
            if (TunerConstants.kCANBus.isNetworkFD()) BaseStatusSignal.waitForAll(0.02, statusSignals);
            else {
                MapleTimeUtils.delay(1.0 / 300.0);
                BaseStatusSignal.refreshAll(statusSignals);
            }
        }

        private double estimateAverageTimeStamps() {
            double currentTime = Timer.getFPGATimestamp(), totalLatency = 0;
            for (BaseStatusSignal signal : statusSignals)
                totalLatency += signal.getTimestamp().getLatency();

            if (statusSignals.length == 0) return currentTime;
            return currentTime - totalLatency / statusSignals.length;
        }

        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.odometryTicksCountInPreviousRobotPeriod = timeStampsQueue.size();
            for (int i = 0; i < ODOMETRY_CACHE_CAPACITY; i++)
                inputs.measurementTimeStamps[i] = Objects.requireNonNullElse(timeStampsQueue.poll(), 0.0);
            timeStampsQueue.clear();
        }

        @Override
        public void lockOdometry() {
            lock.lock();
        }

        @Override
        public void unlockOdometry() {
            lock.unlock();
        }
    }
}
