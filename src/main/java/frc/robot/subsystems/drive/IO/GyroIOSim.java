package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.constants.LogPaths;
import frc.robot.utils.CustomMaths.MapleCommonMath;
import frc.robot.utils.MapleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static frc.robot.constants.DriveTrainConstants.*;

public class GyroIOSim implements GyroIO {
    public final GyroPhysicsSimulationResults gyroPhysicsSimulationResults = new GyroPhysicsSimulationResults();
    public double previousAngularVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec;
    public Rotation2d currentGyroDriftAmount = new Rotation2d();
    public static final String GYRO_LOG_PATH = LogPaths.PHYSICS_SIMULATION_PATH + "GyroSim/";

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        final double angularVelocityChange = Math.abs(
                gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec -  previousAngularVelocityRadPerSec),
                angularAccelerationMagnitudeRadPerSecSq = angularVelocityChange / Robot.defaultPeriodSecs;
        previousAngularVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec;
        final double currentTickDriftStdDevRad = angularAccelerationMagnitudeRadPerSecSq > GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ ?
                angularAccelerationMagnitudeRadPerSecSq * SKIDDING_AMOUNT_AT_THRESHOLD_RAD / GYRO_ANGULAR_ACCELERATION_THRESHOLD_SKIDDING_RAD_PER_SEC_SQ
                : Math.abs(gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec) * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD / AVERAGE_VELOCITY_RAD_PER_SEC_DURING_TEST;
        currentGyroDriftAmount = currentGyroDriftAmount.rotateBy(
                Rotation2d.fromRadians(MapleCommonMath.generateRandomNormal(0, currentTickDriftStdDevRad))
        );

        inputs.connected = gyroPhysicsSimulationResults.hasReading;
        inputs.odometryYawPositions =
                Arrays.stream(gyroPhysicsSimulationResults.odometryYawPositions)
                .map((robotFacing) -> robotFacing.rotateBy(currentGyroDriftAmount))
                .toArray(Rotation2d[]::new);
        inputs.yawPosition = inputs.odometryYawPositions[inputs.odometryYawPositions.length-1];
        inputs.yawVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec;

        Logger.recordOutput(GYRO_LOG_PATH + "robot true yaw (deg)",
                gyroPhysicsSimulationResults.odometryYawPositions[gyroPhysicsSimulationResults.odometryYawPositions.length-1].getDegrees()
        );
        Logger.recordOutput(GYRO_LOG_PATH + "robot power for (Sec)", MapleTimeUtils.getLogTimeSeconds());
        Logger.recordOutput(GYRO_LOG_PATH + "imu total drift (Deg)", currentGyroDriftAmount.getDegrees());
        Logger.recordOutput(GYRO_LOG_PATH + "gyro reading yaw (Deg)", inputs.yawPosition.getDegrees());
        Logger.recordOutput(GYRO_LOG_PATH + "angular velocity (Deg per Sec)", Math.toDegrees(previousAngularVelocityRadPerSec));
        Logger.recordOutput(GYRO_LOG_PATH + "gyro angular acc (Deg per Sec^2)", Math.toDegrees(angularAccelerationMagnitudeRadPerSecSq));
        Logger.recordOutput(GYRO_LOG_PATH + "new drift in current tick Std Dev (Deg)", Math.toDegrees(currentTickDriftStdDevRad));
    }

    /*
    * we know that in one minute, or n=(60 / Robot.defaultPeriodSeconds) periods
    * the gyro's drift has standard deviation of NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
    * sqrt(n) * GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD = NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD
     *  */
    public static final double GYRO_DRIFT_IN_1_TICK_Std_Dev_RAD = NORMAL_GYRO_DRIFT_IN_1_MIN_Std_Dev_RAD / Math.sqrt(60.0/Robot.defaultPeriodSecs);

    public static class GyroPhysicsSimulationResults {
        public double robotAngularVelocityRadPerSec;
        public boolean hasReading;

        public final Rotation2d[] odometryYawPositions =
                new Rotation2d[SIMULATION_TICKS_IN_1_PERIOD];

        public GyroPhysicsSimulationResults() {
            robotAngularVelocityRadPerSec = 0.0;
            hasReading = false;
            Arrays.fill(odometryYawPositions, new Rotation2d());
        }
    }
}
