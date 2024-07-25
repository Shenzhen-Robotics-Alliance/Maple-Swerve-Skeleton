package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.MapleMaths.MapleCommonMath;
import frc.robot.utils.MapleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static frc.robot.Constants.RobotPhysicsSimulationConfigs.*;

public class GyroIOSim implements GyroIO {
    public final GyroPhysicsSimulationResults gyroPhysicsSimulationResults = new GyroPhysicsSimulationResults();
    public double previousAngularVelocityRadPerSec = gyroPhysicsSimulationResults.robotAngularVelocityRadPerSec;
    public Rotation2d currentGyroDriftAmount = new Rotation2d();
    public static final String GYRO_LOG_PATH = Constants.LogConfigs.PHYSICS_SIMULATION_PATH + "GyroSim/";

    @Override
    public void updateInputs(GyroIOInputs inputs) {
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
        Logger.recordOutput(GYRO_LOG_PATH + "robot power for (Sec)" + MapleTimeUtils.getLogTimeSeconds());
        Logger.recordOutput(GYRO_LOG_PATH + "imu total drift (Deg)", currentGyroDriftAmount.getDegrees());
        Logger.recordOutput(GYRO_LOG_PATH + "gyro reading yaw (Deg)" + inputs.yawPosition.getDegrees());
        Logger.recordOutput(GYRO_LOG_PATH + "angular velocity (Deg per Sec)", Math.toDegrees(previousAngularVelocityRadPerSec));

    public static class GyroPhysicsSimulationResults {
        public double robotAngularVelocityRadPerSec;
        public boolean hasReading;

        public final Rotation2d[] odometryYawPositions =
                new Rotation2d[SIM_ITERATIONS_PER_ROBOT_PERIOD];

        public GyroPhysicsSimulationResults() {
            robotAngularVelocityRadPerSec = 0.0;
            hasReading = false;
            Arrays.fill(odometryYawPositions, new Rotation2d());
        }
    }
}
