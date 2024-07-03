// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.utils.MapleTimeUtils;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public class OdometryThread extends Thread {
    public static final class OdometryInput {
        private final Supplier<OptionalDouble> supplier;
        private final Queue<Double> queue;
        private double[] valuesSinceLastRobotPeriod;

        public OdometryInput(Supplier<OptionalDouble> signal) {
            this.supplier = signal;
            this.queue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);
            this.valuesSinceLastRobotPeriod = new double[0];
        }

        public double[] getValuesSinceLastRobotPeriod() {
            return valuesSinceLastRobotPeriod;
        }
    }
    private static final List<OdometryInput> registeredInputs = new ArrayList<>();
    private static final List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();
    public static OdometryInput registerInput(Supplier<OptionalDouble> supplier) {
        final OdometryInput odometryInput = new OdometryInput(supplier);
        registeredInputs.add(odometryInput);
        return odometryInput;
    }
    public static OdometryInput registerSignalInput(StatusSignal<Double> signal) {
        signal.setUpdateFrequency(Constants.ChassisConfigs.ODOMETRY_FREQUENCY, Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS);
        final OdometryInput odometryInput = new OdometryInput(() -> OptionalDouble.of(signal.getValue()));
        registeredStatusSignals.add(signal);
        return odometryInput;
    }

    private interface SignalBlocker {
        void refreshSignalsAndBlockThread();
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

    private final SignalBlocker signalBlocker;
    private final OdometryInput[] odometryInputs;
    private final BaseStatusSignal[] statusSignals;
    private final Queue<Double> timeStampsQueue;
    public OdometryThread(OdometryInput[] odometryInputs, BaseStatusSignal[] statusSignals) {
        this.timeStampsQueue = new ArrayBlockingQueue<>(Constants.ChassisConfigs.ODOMETRY_CACHE_CAPACITY);

        this.odometryInputs = odometryInputs;
        this.statusSignals = statusSignals;

        this.signalBlocker = switch (Constants.ChassisConfigs.chassisType) {
            case REV ->
                    () -> MapleTimeUtils.delay(1.0/ Constants.ChassisConfigs.ODOMETRY_FREQUENCY);
            case CTRE_ON_RIO ->
                    () -> {
                MapleTimeUtils.delay(1.0/ Constants.ChassisConfigs.ODOMETRY_FREQUENCY);
                BaseStatusSignal.refreshAll();
            };
            case CTRE_ON_CANIVORE ->
                    () -> BaseStatusSignal.waitForAll(Constants.ChassisConfigs.ODOMETRY_WAIT_TIMEOUT_SECONDS, statusSignals);
        };

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
        signalBlocker.refreshSignalsAndBlockThread();
        timeStampsQueue.offer(estimateAverageTimeStamps());

        // TODO: for each input poll the reading and cache them in the queue
    }

    private double estimateAverageTimeStamps() {
        double currentTime = MapleTimeUtils.getRealTimeSeconds(), totalLatency = 0;
        for (BaseStatusSignal signal:statusSignals)
            totalLatency += signal.getTimestamp().getLatency();

        if (statusSignals.length == 0)
            return currentTime;
        return currentTime - totalLatency / statusSignals.length;
    }

    public static void updateMainThreadCache() {
        if (instance != null && instance.isAlive())
            instance.
    }

    private double[] odometryTimeStamps; // TODO cache odometry time stamps in main thread and get them whenever ready
    public void updateMainThreadCache() {

    }
}
