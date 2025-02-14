package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.DriveControlLoops.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.DriveTrainConstants;
import frc.robot.utils.AlertsManager;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.PPRobotConfigPrinter;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public interface HolonomicDriveSubsystem extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     *
     * @param speeds a discrete chassis speed, robot-centric
     */
    void runRobotCentricChassisSpeeds(ChassisSpeeds speeds);

    /**
     * runs a ChassisSpeeds without doing any pre-processing
     *
     * @param speeds a discrete chassis speed, robot-centric
     */
    void runRobotCentricSpeedsWithFeedforwards(ChassisSpeeds speeds, DriveFeedforwards feedforwards);

    /** Returns the current odometry Pose. */
    Pose2d getPose();

    default Rotation2d getFacing() {
        return getPose().getRotation();
    }

    /** Resets the current odometry Pose to a given Pose */
    void setPose(Pose2d currentPose);

    /** @return the measured(actual) velocities of the chassis, robot-relative */
    ChassisSpeeds getMeasuredChassisSpeedsRobotRelative();

    default ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    double getChassisMaxLinearVelocityMetersPerSec();

    double getChassisMaxAccelerationMetersPerSecSq();

    double getChassisMaxAngularVelocity();

    double getChassisMaxAngularAccelerationRadPerSecSq();

    default PathConstraints getChassisConstrains(double speedMultiplier) {
        return new PathConstraints(
                getChassisMaxLinearVelocityMetersPerSec() * speedMultiplier,
                getChassisMaxAccelerationMetersPerSecSq() * speedMultiplier * speedMultiplier,
                getChassisMaxAngularVelocity() * speedMultiplier,
                getChassisMaxAngularAccelerationRadPerSecSq() * speedMultiplier * speedMultiplier);
    }

    /**
     * runs a driverstation-centric ChassisSpeeds
     *
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     */
    default void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds, boolean discretize) {
        if (discretize)
            driverStationCentricSpeeds =
                    ChassisSpeeds.discretize(driverStationCentricSpeeds, DISCRETIZE_TIME.in(Seconds));
        final Rotation2d driverStationFacing = FieldMirroringUtils.getCurrentAllianceDriverStationFacing();
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverStationCentricSpeeds, getPose().getRotation().minus(driverStationFacing)));
    }

    /**
     * runs a field-centric ChassisSpeeds
     *
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     */
    default void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds, boolean discretize) {
        if (discretize) fieldCentricSpeeds = ChassisSpeeds.discretize(fieldCentricSpeeds, DISCRETIZE_TIME.in(Seconds));
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds, getPose().getRotation()));
    }

    default void stop() {
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    default RobotConfig defaultPathPlannerRobotConfig() {
        return new RobotConfig(
                DriveTrainConstants.ROBOT_MASS,
                DriveTrainConstants.ROBOT_MOI,
                new ModuleConfig(
                        DriveTrainConstants.WHEEL_RADIUS,
                        DriveTrainConstants.CHASSIS_MAX_VELOCITY,
                        DriveTrainConstants.WHEEL_COEFFICIENT_OF_FRICTION,
                        DriveTrainConstants.DRIVE_MOTOR_MODEL.withReduction(DriveTrainConstants.DRIVE_GEAR_RATIO),
                        DriveTrainConstants.DRIVE_ANTI_SLIP_TORQUE_CURRENT_LIMIT,
                        1),
                DriveTrainConstants.MODULE_TRANSLATIONS);
    }

    default void configHolonomicPathPlannerAutoBuilder(Field2d field) {
        RobotConfig robotConfig = defaultPathPlannerRobotConfig();
        System.out.println("Generated pathplanner robot config with drive constants: ");
        PPRobotConfigPrinter.printConfig(robotConfig);
        try {
            robotConfig = RobotConfig.fromGUISettings();
            System.out.println("GUI robot config detected in deploy directory, switching: ");
            PPRobotConfigPrinter.printConfig(robotConfig);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), false);
        }
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getMeasuredChassisSpeedsRobotRelative,
                this::runRobotCentricSpeedsWithFeedforwards,
                new PPHolonomicDriveController(
                        CHASSIS_TRANSLATION_CLOSE_LOOP.toPathPlannerPIDConstants(),
                        CHASSIS_ROTATION_CLOSE_LOOP.toPathPlannerPIDConstants()),
                robotConfig,
                FieldMirroringUtils::isSidePresentedAsRed,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            final Pose2d[] trajectory = activePath.toArray(new Pose2d[0]);
            Logger.recordOutput("RobotState/Trajectory", trajectory);
            field.getObject("ActivateTrajectory").setPoses(trajectory);
        });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("RobotState/TrajectorySetpoint", targetPose));

        Alert pathPlannerWarmUpInProgressAlert =
                AlertsManager.create("PathPlanner Warm-Up in progress", Alert.AlertType.kWarning);
        pathPlannerWarmUpInProgressAlert.set(true);
        PathfindingCommand.warmupCommand()
                .finallyDo(() -> pathPlannerWarmUpInProgressAlert.set(false))
                .until(DriverStation::isEnabled)
                .schedule();
    }

    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < Math.toRadians(5)
                && Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.05
                && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.05;
    }
}
