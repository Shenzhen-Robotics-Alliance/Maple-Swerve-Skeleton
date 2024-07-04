// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.utils.MapleTimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

public class OdometryThread extends Thread {
    public static final class OdometryInput {
        private final Supplier<Double> supplier;
        private final Queue<Double> queue;
        private Double[] valuesSincePreviousPeriod = new Double[0];

        public OdometryInput(Supplier<Double> signal) {
            this.supplier = signal;
            this.queue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);
        }

        public Double[] getValuesSincePreviousPeriod() {
            return valuesSincePreviousPeriod;
        }
    }
    private static final List<OdometryInput> registeredInputs = new ArrayList<>();
    private static final List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();
    public static OdometryInput registerInput(Supplier<Double> supplier) {
        final OdometryInput odometryInput = new OdometryInput(supplier);
        registeredInputs.add(odometryInput);
        return odometryInput;
    }
    public static OdometryInput registerSignalInput(StatusSignal<Double> signal) {
        signal.setUpdateFrequency(Constants.ChassisConfigs.ODOMETRY_FREQUENCY, Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS);
        final OdometryInput odometryInput = new OdometryInput(signal.asSupplier());
        registeredStatusSignals.add(signal);
        return odometryInput;
    }

    private static OdometryThread instance = null;
    public static OdometryThread getInstance() {
        if (instance == null)
            instance = new OdometryThread(
                    registeredInputs.toArray(new OdometryInput[0]),
                    registeredStatusSignals.toArray(new BaseStatusSignal[0])
            );
        return instance;
    }

    private final OdometryInput[] odometryInputs;
    private final BaseStatusSignal[] statusSignals;
    private final Queue<Double> timeStampsQueue;
    private Double[] timeStamps = new Double[0];
    private final Lock odometryLock = new ReentrantLock();
    public OdometryThread(OdometryInput[] odometryInputs, BaseStatusSignal[] statusSignals) {
        this.timeStampsQueue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);

        this.odometryInputs = odometryInputs;
        this.statusSignals = statusSignals;

        setName("OdometryThread");
        setDaemon(true);
    }

    @Override
    public synchronized void start() {
        if (odometryInputs.length > 0)
            super.start();
    }

    public static Double[] getOdometryTimeStamps() {
        if (instance == null || !instance.isAlive())
            return new Double[0];
        return instance.getTimeStamps();
    }
    private Double[] getTimeStamps() {
        return timeStamps;
    }

    @Override
    public void run() {
        while (true) pollInputsInBackEnd();
    }

    private void pollInputsInBackEnd() {
        refreshSignalsAndBlockThread();

        odometryLock.lock();
        timeStampsQueue.offer(estimateAverageTimeStamps());
        for(OdometryInput odometryInput:odometryInputs)
            odometryInput.queue.offer(odometryInput.supplier.get());
        odometryLock.unlock();
    }

    private void refreshSignalsAndBlockThread() {
        switch (Constants.ChassisConfigs.chassisType) {
            case REV -> MapleTimeUtils.delay(1.0 / Constants.ChassisConfigs.ODOMETRY_FREQUENCY);
            case CTRE_ON_RIO -> {
                MapleTimeUtils.delay(1.0 / Constants.ChassisConfigs.ODOMETRY_FREQUENCY);
                BaseStatusSignal.refreshAll();
            }
            case CTRE_ON_CANIVORE -> BaseStatusSignal.waitForAll(Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS, statusSignals);
        }
    }

    private double estimateAverageTimeStamps() {
        double currentTime = MapleTimeUtils.getRealTimeSeconds(), totalLatency = 0;
        for (BaseStatusSignal signal:statusSignals)
            totalLatency += signal.getTimestamp().getLatency();

        if (statusSignals.length == 0)
            return currentTime;
        return currentTime - totalLatency / statusSignals.length;
    }

    public static void fetchOdometryDataSincePreviousRobotPeriod() {
        if (instance != null && instance.isAlive())
            instance.fetchDataSincePreviousRobotPeriod();
    }
    private void fetchDataSincePreviousRobotPeriod() {
        odometryLock.lock();
        this.timeStamps = mapQueueToArray(timeStampsQueue);
        timeStampsQueue.clear();

        for(OdometryInput odometryInput:odometryInputs) {
            odometryInput.valuesSincePreviousPeriod = mapQueueToArray(odometryInput.queue);
            odometryInput.queue.clear();
        }

        odometryLock.unlock();
    }

    private static Double[] mapQueueToArray(Queue<Double> queue) {
        return queue.toArray(new Double[0]);
    }
}
