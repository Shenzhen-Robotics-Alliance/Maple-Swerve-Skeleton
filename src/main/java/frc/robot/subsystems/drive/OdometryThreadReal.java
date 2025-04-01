// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/
package frc.robot.subsystems.drive;

import static frc.robot.constants.DriveTrainConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.MapleTimeUtils;
import java.util.Objects;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThreadReal extends Thread implements OdometryThread {
    private final SwerveDrive.DriveType driveType;

    private final OdometryInput[] odometryDoubleInputs;
    private final BaseStatusSignal[] statusSignals;
    private final Queue<Double> timeStampsQueue;
    private final Lock lock = new ReentrantLock();

    public OdometryThreadReal(
            SwerveDrive.DriveType driveType, OdometryInput[] odometryInputs, BaseStatusSignal[] statusSignals) {
        this.driveType = driveType;
        this.timeStampsQueue = new ArrayBlockingQueue<>(ODOMETRY_CACHE_CAPACITY);
        this.odometryDoubleInputs = odometryInputs;
        this.statusSignals = statusSignals;

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
        switch (driveType) {
            case GENERIC -> MapleTimeUtils.delay(1.0 / ODOMETRY_FREQUENCY);
            case CTRE -> {
                MapleTimeUtils.delay(1.0 / ODOMETRY_FREQUENCY);
                BaseStatusSignal.refreshAll(statusSignals);
            }
            case CTRE_TIME_SYNCHRONIZED -> BaseStatusSignal.waitForAll(ODOMETRY_WAIT_TIMEOUT_SECONDS, statusSignals);
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
