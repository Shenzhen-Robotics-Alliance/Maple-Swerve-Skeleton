package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.ChassisHeadingController;
import frc.robot.utils.MapleJoystickDriveInput;
import java.util.function.DoubleSupplier;
import org.ironmaple.utils.FieldMirroringUtils;

public class ClockDrive extends Command {
    public static final double ROTATION_AXIS_THRESHOLD = 0.5;

    private final HolonomicDriveSubsystem driveSubsystem;
    private final MapleJoystickDriveInput input;
    private final DoubleSupplier rotationXSupplier, rotationYSupplier;

    private Rotation2d currentDesiredFacing;

    public ClockDrive(
            HolonomicDriveSubsystem driveSubsystem,
            MapleJoystickDriveInput input,
            DoubleSupplier rotationXSupplier,
            DoubleSupplier rotationYSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.input = input;
        this.rotationXSupplier = rotationXSupplier;
        this.rotationYSupplier = rotationYSupplier;

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        currentDesiredFacing = driveSubsystem.getFacing();
    }

    @Override
    public void execute() {
        final ChassisSpeeds pilotInputSpeed = input.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(),
                driveSubsystem.getChassisMaxAngularVelocity());

        Translation2d headingVector =
                new Translation2d(rotationXSupplier.getAsDouble(), rotationYSupplier.getAsDouble());
        if (headingVector.getNorm() > ROTATION_AXIS_THRESHOLD)
            this.currentDesiredFacing =
                    headingVector.getAngle().plus(FieldMirroringUtils.getCurrentAllianceDriverStationFacing());

        ChassisHeadingController.getInstance()
                .setHeadingRequest(new ChassisHeadingController.FaceToRotationRequest(this.currentDesiredFacing));

        driveSubsystem.runDriverStationCentricChassisSpeeds(pilotInputSpeed, true);
    }

    @Override
    public void end(boolean interrupted) {
        ChassisHeadingController.getInstance().setHeadingRequest(new ChassisHeadingController.NullRequest());
    }
}
