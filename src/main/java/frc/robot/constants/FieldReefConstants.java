package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Arrays;
import org.ironmaple.utils.FieldMirroringUtils;

public class FieldReefConstants {
    public record BranchTarget(Pose2d roughApproachPose, Pose2d preciseAlignmentPose, int tagId) {}
    private static int selectedId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static final Pose2d[] REEF_ALIGNMENT_POSITIONS_BLUE = new Pose2d[] {
        new Pose2d(3.45, 4.15, Rotation2d.fromDegrees(0)),
        new Pose2d(3.45, 3.74, Rotation2d.fromDegrees(0)),
        new Pose2d(3.82, 3.19, Rotation2d.fromDegrees(60)),
        new Pose2d(4.16, 3.00, Rotation2d.fromDegrees(60)),
        new Pose2d(4.86, 2.98, Rotation2d.fromDegrees(120)),
        new Pose2d(5.18, 3.16, Rotation2d.fromDegrees(120)),
        new Pose2d(5.51, 3.88, Rotation2d.fromDegrees(180)),
        new Pose2d(5.51, 4.17, Rotation2d.fromDegrees(180)),
        new Pose2d(5.08, 4.85, Rotation2d.fromDegrees(-120)),
        new Pose2d(4.93, 5.01, Rotation2d.fromDegrees(-120)),
        new Pose2d(4.05, 5.04, Rotation2d.fromDegrees(-60)),
        new Pose2d(3.73, 4.79, Rotation2d.fromDegrees(-60)),
    };

    public static final Pose2d[] REEF_ALIGNMENT_POSITIONS_RED = new Pose2d[] {
        new Pose2d(14.1, 3.9, Rotation2d.fromDegrees(180)),
        new Pose2d(14.1, 4.31, Rotation2d.fromDegrees(180)),
        new Pose2d(13.73, 4.86, Rotation2d.fromDegrees(-120)),
        new Pose2d(13.39, 5.05, Rotation2d.fromDegrees(-120)),
        new Pose2d(12.69, 5.07, Rotation2d.fromDegrees(-60)),
        new Pose2d(12.37, 4.89, Rotation2d.fromDegrees(-60)),
        new Pose2d(12.04, 4.17, Rotation2d.fromDegrees(0)),
        new Pose2d(12.04, 3.88, Rotation2d.fromDegrees(0)),
        new Pose2d(12.47, 3.20, Rotation2d.fromDegrees(60)),
        new Pose2d(12.62, 3.04, Rotation2d.fromDegrees(60)),
        new Pose2d(13.50, 3.01, Rotation2d.fromDegrees(120)),
        new Pose2d(13.82, 3.26, Rotation2d.fromDegrees(120)),
    };

    public static Pose2d getReefAlignmentTargetPose() {
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[selectedId]
                : REEF_ALIGNMENT_POSITIONS_BLUE[selectedId];
    }

    public static boolean[] displayReefTarget() {
        boolean[] reef = new boolean[12];
        Arrays.fill(reef, false);
        reef[selectedId] = true;
        return reef;
    }

    public static void nextTarget() {
        if (++selectedId >= 12) selectedId = 0;
    }

    public static void previousTarget() {
        if (--selectedId < 0) selectedId = 11;
    }

    public Command nextTargetButton(double debugTime) {
        return Commands.runOnce(FieldReefConstants::nextTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public Command previousTargetButton(double debugTime) {
        return Commands.runOnce(FieldReefConstants::nextTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }
}
