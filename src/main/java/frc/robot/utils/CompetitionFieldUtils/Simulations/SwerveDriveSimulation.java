package frc.robot.utils.CompetitionFieldUtils.Simulations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.constants.LogPaths;
import frc.robot.subsystems.drive.IO.GyroIOSim;
import frc.robot.subsystems.drive.IO.ModuleIOSim;
import frc.robot.subsystems.drive.IO.OdometryThread;
import frc.robot.utils.MapleTimeUtils;

import java.util.Arrays;
import java.util.function.Consumer;

import static frc.robot.constants.DriveTrainConstants.*;

/**
 * simulates the behavior of our robot
 * it has all the physics behavior as a simulated holonomic chassis
 * in addition to that, it simulates the swerve module behaviors
 * the class is like the bridge between ModuleIOSim and HolonomicChassisSimulation
 * it reads the motor power from ModuleIOSim
 * and feed the result of the physics simulation back to ModuleIOSim, to simulate the odometry encoders' readings
 * */
public class SwerveDriveSimulation extends HolonomicChassisSimulation {
    private static final String LOG_PATH = LogPaths.PHYSICS_SIMULATION_PATH + "SwerveDriveSim/";

    private final GyroIOSim gyroIOSim;
    private final ModuleIOSim[] modules;
    private final Consumer<Pose2d> resetOdometryCallBack;
    public SwerveDriveSimulation(
            GyroIOSim gyroIOSim,
            ModuleIOSim frontLeft, ModuleIOSim frontRight, ModuleIOSim backLeft, ModuleIOSim backRight,
            Pose2d startingPose,
            Consumer<Pose2d> resetOdometryCallBack
    ) {
        super(new RobotSimulationProfile(

        ), startingPose);
        this.gyroIOSim = gyroIOSim;
        this.modules = new ModuleIOSim[] {frontLeft, frontRight, backLeft, backRight};
        this.resetOdometryCallBack = resetOdometryCallBack;
        resetOdometryToActualRobotPose();
        System.out.println("swerve drive sim profile: " + new RobotSimulationProfile());
    }

    public void resetOdometryToActualRobotPose() {
        resetOdometryCallBack.accept(getObjectOnFieldPose2d());
    }

    @Override
    public void updateSimulationSubTick(int tickNum, double tickSeconds) {
        for (int i = 0; i < modules.length; i++)
            moduleSimulationSubTick(
                    modules[i],
                    MODULE_TRANSLATIONS[i],
                    tickNum, tickSeconds
            );

        final ChassisSpeeds chassisFreeSpeeds = DRIVE_KINEMATICS.toChassisSpeeds(Arrays.stream(modules).map((
                moduleIOSim -> new SwerveModuleState(
                        moduleIOSim.physicsSimulationResults.driveWheelFinalVelocityRadPerSec * WHEEL_RADIUS_METERS,
                        moduleIOSim.getSteerFacing()
                ))).toArray(SwerveModuleState[]::new));

        // TODO: here we apply the friction force that brings the chassis from its floor speed to free speed
        

        gyroSimulationSubTick(
                gyroIOSim,
                super.getObjectOnFieldPose2d().getRotation(),
                super.getAngularVelocity(),
                tickNum
        );
    }

    private static void moduleSimulationSubTick(
            ModuleIOSim module,
            Translation2d moduleTranslation,
            int tickNum,
            double tickPeriodSeconds
    ) {
        // TODO: here, apply the propelling force of each module on the robot
        //  and feed its odometry drive position and velocity back to the results
    }

    private static void gyroSimulationSubTick(
            GyroIOSim gyroIOSim,
            Rotation2d currentFacing,
            double angularVelocityRadPerSec,
            int tickNum
    ) {
        final GyroIOSim.GyroPhysicsSimulationResults results = gyroIOSim.gyroPhysicsSimulationResults;
        results.robotAngularVelocityRadPerSec = angularVelocityRadPerSec;
        results.odometryYawPositions[tickNum] = currentFacing;
        results.hasReading = true;
    }

    public static final class OdometryThreadSim implements OdometryThread {
        @Override
        public void updateInputs(OdometryThreadInputs inputs) {
            inputs.measurementTimeStamps = new double[SIMULATION_TICKS_IN_1_PERIOD];
            final double robotStartingTimeStamps = MapleTimeUtils.getLogTimeSeconds(),
                    iterationPeriodSeconds = Robot.defaultPeriodSecs/SIMULATION_TICKS_IN_1_PERIOD;
            for (int i =0; i < SIMULATION_TICKS_IN_1_PERIOD; i++)
                inputs.measurementTimeStamps[i] = robotStartingTimeStamps + i * iterationPeriodSeconds;
        }
    }
}
