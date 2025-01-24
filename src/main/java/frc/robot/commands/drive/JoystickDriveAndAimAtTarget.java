package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import java.util.function.Supplier;

/**
 *
 *
 * <h1>Custom Drive Command</h1>
 *
 * <p>The chassis will automatically face to a target on field (eg. the speaker) while the pilot controls its movements
 * The chassis will also adjust its facing in-advance, with respect to the flight time calculated from
 * {@link MapleShooterOptimization} (this is for shooting-on-the-move)
 */
public class JoystickDriveAndAimAtTarget {
    public static Command driveAndAimAtTarget(
            MapleJoystickDriveInput input,
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Translation2d> targetPositionSupplier,
            MapleShooterOptimization shooterOptimization,
            double pilotInputMultiplier,
            boolean finishWhenComplete) {
        return new FunctionalCommand(
                () -> ChassisHeadingController.getInstance()
                        .setHeadingRequest(new ChassisHeadingController.FaceToTargetRequest(
                                targetPositionSupplier, shooterOptimization)),
                () -> execute(driveSubsystem, input, pilotInputMultiplier),
                (interrupted) -> ChassisHeadingController.getInstance()
                        .setHeadingRequest(new ChassisHeadingController.NullRequest()),
                () -> finishWhenComplete
                        && ChassisHeadingController.getInstance().atSetPoint(),
                driveSubsystem);
    }

    public static Command driveAndAimAtDirection(
            MapleJoystickDriveInput input,
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Rotation2d> rotationTarget,
            double pilotInputMultiplier,
            boolean finishWhenComplete) {
        return new FunctionalCommand(
                () -> ChassisHeadingController.getInstance()
                        .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(rotationTarget.get())),
                () -> execute(driveSubsystem, input, pilotInputMultiplier),
                (interrupted) -> ChassisHeadingController.getInstance()
                        .setHeadingRequest(new ChassisHeadingController.NullRequest()),
                () -> finishWhenComplete
                        && ChassisHeadingController.getInstance().atSetPoint(),
                driveSubsystem);
    }

    public static void execute(
            HolonomicDriveSubsystem driveSubsystem, MapleJoystickDriveInput input, double pilotInputMultiplier) {
        driveSubsystem.runDriverStationCentricChassisSpeeds(
                input.getJoystickChassisSpeeds(
                        driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * pilotInputMultiplier, 0),
                true);
    }
}
