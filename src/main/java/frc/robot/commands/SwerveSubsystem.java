package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveTrainConfigs.*;
import static frc.robot.subsystems.drive.DriveTrainConstants.*;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public interface SwerveSubsystem extends Subsystem {
    void stop();

    void executeSetpoint(SwerveSetpoint setpoint);

    void runCharacterization(Voltage voltage);

    static void resetSwerveSetpoint() {
        RobotState.getInstance().setpoint = new SwerveSetpoint(
                RobotState.getInstance().getRobotRelativeSpeeds(),
                RobotState.getInstance().getMeasuredStates(),
                DriveFeedforwards.zeros(4));
    }

    default Command driveUsingSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier) {
        return run(() -> {
                    RobotState.getInstance().setpoint = setpointGenerator.generateSetpoint(
                            RobotState.getInstance().setpoint,
                            robotRelativeSpeedsSupplier.get(),
                            RobotState.getInstance().lowSpeedModeEnabled()
                                    ? DRIVE_CONSTRAINS_LOW_SPEED
                                    : DRIVE_CONSTRAINS,
                            Robot.defaultPeriodSecs,
                            12.0);
                    executeSetpoint(RobotState.getInstance().setpoint);
                })
                .beforeStarting(SwerveSubsystem::resetSwerveSetpoint)
                .finallyDo(this::stop);
    }

    default void driveWithFeedforward(ChassisSpeeds robotRelativeSpeeds, DriveFeedforwards feedforwards) {
        SwerveSetpoint setpoint = new SwerveSetpoint(
                robotRelativeSpeeds, DRIVE_KINEMATICS.toSwerveModuleStates(robotRelativeSpeeds), feedforwards);
        executeSetpoint(setpoint);
    }

    static void configurePathPlannerLogging(Field2d field) {
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            field.getObject("ActivePath").setPoses(poses);
            Logger.recordOutput("RobotState/ActivePath", poses.toArray(Pose2d[]::new));
        });
        PathPlannerLogging.setLogTargetPoseCallback(
                (pose) -> Logger.recordOutput("RobotState/TrajectoryTargetPose", pose));
    }

    default Command followPath(PathPlannerPath path) {
        return new FollowPathCommand(
                        path,
                        RobotState.getInstance()::getPoseWithLookAhead,
                        RobotState.getInstance()::getRobotRelativeSpeeds,
                        this::driveWithFeedforward,
                        PP_DRIVE_CONTROLLER,
                        pathplannerRobotConfig(),
                        FieldMirroringUtils::isSidePresentedAsRed,
                        this)
                .finallyDo(this::stop)
                .finallyDo(SwerveSubsystem::resetSwerveSetpoint);
    }

    static SwerveModuleState[] xFormationStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = new SwerveModuleState(0.0, MODULE_TRANSLATIONS[i].getAngle());

        return states;
    }

    default Command lockWithXFormation() {
        return run(() -> executeSetpoint(
                new SwerveSetpoint(new ChassisSpeeds(), xFormationStates(), DriveFeedforwards.zeros(4))));
    }

    default SysIdRoutine sysIdRoutine() {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism(this::runCharacterization, null, this));
    }

    /*
     * To run characterization for swerve:
     *   1. Select SysId command on dashboard and run robot test.
     *   2. Export the log file using AdvantageScope, (Format: "WPILOG", Timestamps:"AdvantageKit Cycles", Prefixes:"Drive/,RealOutputs/Drive/SysIdState").
     *   3. Open the exported log file with Wpilib SysId Tool, select the data:
     *       - Test State: RealOutputs/Drive/SysIdState
     *       - Velocity: Drive/Module-MODULE_NAME/DriveWheelFinalVelocityRevolutionsPerSecond
     *       - Position: Drive/Module-MODULE_NAME/DriveWheelFinalRevolutions
     *       - Voltage: Drive/Module-MODULE_NAME/DriveAppliedVolts
     *   4. Calculate the gains for ALL FOUR modules, take the average.
     *       - Note that if the difference between modules are too big, SOMETHING IS WRONG.
     *   4. The calculated kS is correct; but kV NEEDS TO BE DIVIDED BY GEAR RATIO.
     *   5. Don't use the calculated kP, tune the kP manually.
     * */

    default Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine().quasistatic(direction);
    }

    default Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine().dynamic(direction);
    }
}
