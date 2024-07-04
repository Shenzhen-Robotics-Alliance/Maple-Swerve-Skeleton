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

        public OdometryInput(Supplier<Double> signal) {
            this.supplier = signal;
            this.queue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);
        }

    }
    private static final List<OdometryInput> registeredInputs = new ArrayList<>();
    private static final List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();
    public static Queue<Double> registerInput(Supplier<Double> supplier) {
        final OdometryInput odometryInput = new OdometryInput(supplier);
        registeredInputs.add(odometryInput);
        return odometryInput.queue;
    }
    public static Queue<Double> registerSignalInput(StatusSignal<Double> signal) {
        signal.setUpdateFrequency(Constants.ChassisConfigs.ODOMETRY_FREQUENCY, Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS);
        final OdometryInput odometryInput = new OdometryInput(signal.asSupplier());
        registeredStatusSignals.add(signal);
        return odometryInput.queue;
    }

    private static OdometryThread instance = null;
    private static OdometryThread getInstance() {
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
    private Double[] odometryTimeStamps = new Double[0];
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

    public static void pollOdometryTimeStampsDuringPreviousRobotPeriod() {
        if (instance != null && instance.isAlive())
            instance.pollTimeStampsDuringPreviousRobotPeriod();
    }
    public void pollTimeStampsDuringPreviousRobotPeriod() {
        this.odometryTimeStamps = timeStampsQueue.toArray(new Double[0]);
    }
}
