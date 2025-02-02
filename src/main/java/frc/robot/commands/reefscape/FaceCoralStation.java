package frc.robot.commands.reefscape;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.JoystickDriveAndAimAtTarget;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import org.ironmaple.utils.FieldMirroringUtils;

public class FaceCoralStation {
    public static Command faceCoralStation(HolonomicDriveSubsystem drive, MapleJoystickDriveInput driveInput) {
        return JoystickDriveAndAimAtTarget.driveAndAimAtDirection(
                driveInput,
                drive,
                () -> {
                    boolean onUpperSideOfField =
                            drive.getPose().getTranslation().getY() > 4.01;
                    boolean lefterStationCloser = onUpperSideOfField ^ FieldMirroringUtils.isSidePresentedAsRed();
                    return FieldMirroringUtils.toCurrentAllianceRotation(
                            lefterStationCloser ? Rotation2d.fromDegrees(-53) : Rotation2d.fromDegrees(53));
                },
                0.5,
                false);
    }
}
