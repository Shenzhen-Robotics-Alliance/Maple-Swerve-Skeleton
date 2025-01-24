package frc.robot.commands.tmp;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.commands.drive.JoystickDrive;
import java.util.Set;
import org.ironmaple.utils.FieldMirroringUtils;

public class FaceCoralStation {
    public static Command faceCoralStation() {
        return Commands.defer(
                () -> {
                    boolean onUpperSideOfField = RobotState.getInstance()
                                    .getEstimatedPose()
                                    .getTranslation()
                                    .getY()
                            > 4.01;
                    boolean lefterStationCloser = onUpperSideOfField ^ FieldMirroringUtils.isSidePresentedAsRed();
                    final Rotation2d rotationTarget = FieldMirroringUtils.toCurrentAllianceRotation(
                            lefterStationCloser ? Rotation2d.fromDegrees(-53) : Rotation2d.fromDegrees(53));
                    return Commands.startEnd(
                            () -> JoystickDrive.instance.ifPresent(joystickDrive -> {
                                joystickDrive.setRotationMaintenanceSetpoint(rotationTarget);
                                joystickDrive.setSensitivity(0.5, 0);
                            }),
                            () -> JoystickDrive.instance.ifPresent(
                                    joystickDrive -> joystickDrive.setSensitivity(1, 1)));
                },
                Set.of());
    }
}
