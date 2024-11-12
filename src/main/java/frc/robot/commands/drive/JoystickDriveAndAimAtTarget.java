package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.drive.SwerveDrive;
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
public class JoystickDriveAndAimAtTarget extends Command {
    private final MapleJoystickDriveInput input;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final MapleShooterOptimization shooterOptimization;
    private final HolonomicDriveSubsystem driveSubsystem;

    private final double pilotInputMultiplier;

    public JoystickDriveAndAimAtTarget(
            MapleJoystickDriveInput input,
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Translation2d> targetPositionSupplier,
            MapleShooterOptimization shooterOptimization,
            double pilotInputMultiplier) {
        this.targetPositionSupplier = targetPositionSupplier;
        this.shooterOptimization = shooterOptimization;
        this.pilotInputMultiplier = pilotInputMultiplier;

        this.driveSubsystem = driveSubsystem;
        this.input = new MapleJoystickDriveInput(input.joystickXSupplier, input.joystickYSupplier, () -> 0);

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        SwerveDrive.swerveHeadingController.setHeadingRequest(
                new ChassisHeadingController.FaceToTargetRequest(targetPositionSupplier, shooterOptimization));
    }

    @Override
    public void execute() {
        driveSubsystem.runDriverStationCentricChassisSpeeds(
                input.getJoystickChassisSpeeds(
                        driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * pilotInputMultiplier, 0),
                true);
    }

    public boolean chassisRotationInPosition() {
        return SwerveDrive.swerveHeadingController.atSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDrive.swerveHeadingController.setHeadingRequest(new ChassisHeadingController.NullRequest());
    }
}
