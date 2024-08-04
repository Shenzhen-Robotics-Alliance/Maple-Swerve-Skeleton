package frc.robot.utils.CompetitionFieldUtils.Simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.Config.MapleConfigFile;
import frc.robot.utils.MapleMaths.SwerveStateProjection;
import frc.robot.utils.MapleTimeUtils;
import org.dyn4j.dynamics.Force;
import org.dyn4j.geometry.Vector2;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.Consumer;

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
    private static final String LOG_PATH = Constants.LogConfigs.PHYSICS_SIMULATION_PATH + "SwerveDriveSim/";

    private final GyroIOSim gyroIOSim;
    private final ModuleIOSim[] modules;
    private final SwerveDriveKinematics kinematics;
    private final Consumer<Pose2d> resetOdometryCallBack;
    public SwerveDriveSimulation(
            MapleConfigFile.ConfigBlock chassisGeneralInfoBlock,
            GyroIOSim gyroIOSim,
            ModuleIOSim frontLeft, ModuleIOSim frontRight, ModuleIOSim backLeft, ModuleIOSim backRight,
            SwerveDriveKinematics kinematics,
            Pose2d startingPose,
            Consumer<Pose2d> resetOdometryCallBack
    ) {
        super(new RobotProfile(chassisGeneralInfoBlock), startingPose);
        this.gyroIOSim = gyroIOSim;
        this.modules = new ModuleIOSim[] {frontLeft, frontRight, backLeft, backRight};
        this.kinematics = kinematics;
        this.resetOdometryCallBack = resetOdometryCallBack;
        resetOdometryToActualRobotPose();
        System.out.println("swerve drive sim profile: " + new RobotProfile(chassisGeneralInfoBlock));
    }

    public void resetOdometryToActualRobotPose() {
        resetOdometryCallBack.accept(getObjectOnFieldPose2d());
    }

    @Override
    public void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds) {
        for (ModuleIOSim module:modules)
            module.updateSim(subPeriodSeconds);
        final ChassisSpeeds swerveWheelFreeSpeeds = kinematics.toChassisSpeeds(
                Arrays.stream(modules)
                        .map(moduleIOSim -> moduleIOSim.getFreeSwerveSpeed(profile.robotMaxVelocity))
                        .toArray(SwerveModuleState[]::new)
        );
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(swerveWheelFreeSpeeds);

        final ChassisSpeeds instantVelocityRobotRelative = getMeasuredChassisSpeedsRobotRelative();
        final SwerveModuleState[] actualModuleFloorSpeeds = kinematics.toSwerveModuleStates(instantVelocityRobotRelative);

        updateGyroSimulationResults(
                gyroIOSim,
                super.getObjectOnFieldPose2d().getRotation(),
                super.getAngularVelocity(),
                iterationNum
        );
        for (int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++)
            updateModuleSimulationResults(
                    modules[moduleIndex],
                    actualModuleFloorSpeeds[moduleIndex],
                    profile.robotMaxVelocity,
                    iterationNum, subPeriodSeconds
            );
    }

    private final Vector2 previousDesiredLinearMotionPercent = new Vector2();
    @Override
    protected void simulateChassisTranslationalBehavior(Vector2 desiredLinearMotionPercent) {
        super.simulateChassisTranslationalBehavior(desiredLinearMotionPercent);
        
        final double dTheta = previousDesiredLinearMotionPercent.getAngleBetween(desiredLinearMotionPercent),
                desiredMotionDirectionChangingRateOmega = dTheta / (Robot.defaultPeriodSecs / SIM_ITERATIONS_PER_ROBOT_PERIOD),
                centripetalForce = desiredMotionDirectionChangingRateOmega * getLinearVelocity().getMagnitude() * profile.robotMass;

        super.applyForce(Vector2.create(
                MathUtil.clamp(centripetalForce, -profile.frictionForce, profile.frictionForce),
                getLinearVelocity().getDirection() + Math.toRadians(90)
        ));
        previousDesiredLinearMotionPercent.set(desiredLinearMotionPercent);
    }

    private static void updateGyroSimulationResults(
            GyroIOSim gyroIOSim,
            Rotation2d currentFacing,
            double angularVelocityRadPerSec,
            int iterationNum) {
        final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
        results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
        results.odometryYawPositions[iterationNum] = currentFacing;
        results.hasReading = true;
    }

    private static void updateModuleSimulationResults(
            ModuleIOSim module,
            SwerveModuleState actualModuleFloorSpeed,
            double robotMaxVelocity,
            int simulationIteration, double periodSeconds) {
        final SwerveModuleState moduleFreeSwerveSpeed = module.getFreeSwerveSpeed(robotMaxVelocity);
        final ModuleIOSim.SwerveModulePhysicsSimulationResults results = module.physicsSimulationResults;
        final double projectedModuleFloorSpeedMetersPerSecond = SwerveStateProjection.project(
                actualModuleFloorSpeed,
                moduleFreeSwerveSpeed.angle
        );

        results.driveWheelFinalVelocityRevolutionsPerSec = getActualDriveMotorRotterSpeedRevPerSec(
                projectedModuleFloorSpeedMetersPerSecond,
                moduleFreeSwerveSpeed.speedMetersPerSecond
        );
        results.odometrySteerPositions[simulationIteration] = moduleFreeSwerveSpeed.angle;
        results.driveWheelFinalRevolutions += results.driveWheelFinalVelocityRevolutionsPerSec * periodSeconds;
        results.odometryDriveWheelRevolutions[simulationIteration] = results.driveWheelFinalRevolutions;
    }


    private static double getActualDriveMotorRotterSpeedRevPerSec(double moduleSpeedProjectedOnSwerveHeadingMPS, double moduleFreeSpeedMPS) {
        // TODO: move configs to Constants
        final double FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED = 0.8, rotterSpeedMPS;
        if (Math.abs(moduleFreeSpeedMPS - moduleSpeedProjectedOnSwerveHeadingMPS) < 2)
            rotterSpeedMPS = moduleSpeedProjectedOnSwerveHeadingMPS;
        else rotterSpeedMPS =
                moduleSpeedProjectedOnSwerveHeadingMPS * FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED
                        + moduleFreeSpeedMPS * (1 - FLOOR_SPEED_WEIGHT_IN_ACTUAL_MOTOR_SPEED);

        final double rotterSpeedRadPerSec = rotterSpeedMPS / DEFAULT_WHEEL_RADIUS_METERS;
        return Units.radiansToRotations(rotterSpeedRadPerSec);
    }

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.measurementTimeStamps = new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
            final double robotStartingTimeStamps = MapleTimeUtils.getLogTimeSeconds(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs/SIM_ITERATIONS_PER_ROBOT_PERIOD;
            for (int i =0; i < SIM_ITERATIONS_PER_ROBOT_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
