package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.SwerveStateProjection;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static frc.robot.Constants.RobotPhysicsSimulationConfigs.*;
import static frc.robot.Constants.ChassisDefaultConfigs.*;

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 * */
public class SwerveDriveSimulation extends HolonomicChassisSimulation {
    private final ModuleIOSim[] modules;
    private final SwerveDriveKinematics kinematics;
    public SwerveDriveSimulation(
            MapleConfigFile.ConfigBlock chassisGeneralInfoBlock,
            ModuleIOSim frontLeft, ModuleIOSim frontRight, ModuleIOSim backLeft, ModuleIOSim backRight,
            SwerveDriveKinematics kinematics,
            Pose2d startingPose) {
        super(new RobotProfile(chassisGeneralInfoBlock), startingPose);
        this.modules = new ModuleIOSim[] {frontLeft, frontRight, backLeft, backRight};
        this.kinematics = kinematics;
    }

    @Override
    public void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds) {
        for (ModuleIOSim module:modules)
            module.updateSim(subPeriodSeconds);
        final ChassisSpeeds swerveWheelsSpeeds = kinematics.toChassisSpeeds(
                Arrays.stream(modules)
                        .map(ModuleIOSim::getFreeSwerveSpeed)
                        .toArray(SwerveModuleState[]::new)
        );
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(swerveWheelsSpeeds);

        final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
        final SwerveModuleState[] actualModuleFloorSpeeds = kinematics.toSwerveModuleStates(instantVelocityRobotRelative);

        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++)
            updateSimulationResults(actualModuleFloorSpeeds[moduleIndex], modules[moduleIndex], iterationNum, subPeriodSeconds);
    }

    private static void updateSimulationResults(
            SwerveModuleState actualModuleFloorSpeed,
            ModuleIOSim module,
            int simulationIteration, double periodSeconds) {
        final ModuleIOSim.SwerveModulePhysicsSimulationResults results = module.physicsSimulationResults;
        final double projectedModuleFloorSpeedMetersPerSecond = SwerveStateProjection.project(
                actualModuleFloorSpeed,
                module.getFreeSwerveSpeed().angle
        );

        results.driveWheelFinalVelocityRevolutionsPerSec = getActualDriveMotorRotterSpeedRevPerSec(
                projectedModuleFloorSpeedMetersPerSecond,
                module.getFreeSwerveSpeed().speedMetersPerSecond
        );
        results.odometrySteerPositions[simulationIteration] = module.getFreeSwerveSpeed().angle;
        results.driveWheelFinalRevolutions += results.driveWheelFinalVelocityRevolutionsPerSec * periodSeconds;
        results.odometryDriveWheelRevolutions[simulationIteration] = results.driveWheelFinalRevolutions;
    }


    private static double getActualDriveMotorRotterSpeedRevPerSec(double moduleSpeedProjectedOnSwerveHeadingMPS, double moduleFreeSpeedMPS) {
        final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = 0.8,
                speedMPS = moduleSpeedProjectedOnSwerveHeadingMPS * FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED
                        + moduleFreeSpeedMPS * (1 - FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED),
                speedRadPerSec = speedMPS / DEFAULT_WHEEL_RADIUS_METERS;
        return Units.radiansToRotations(speedRadPerSec);
    }

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.measurementTimeStamps = new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
            final double robotStartingTimeStamps = Logger.getTimestamp(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs/SIM_ITERATIONS_PER_ROBOT_PERIOD;
            for (int i =0; i < SIM_ITERATIONS_PER_ROBOT_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
