package frc.robot.commands.reefscape;

import static frc.robot.constants.ReefConstants.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.PathUtils;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefAlignment {
    public record BranchTarget(
            Rotation2d facing, Translation2d roughApproachPosition, Translation2d preciseAlignmentPosition, int tagId) {

        public Pose2d roughApproachPose() {
            return new Pose2d(roughApproachPosition, facing);
        }

        public Pose2d preciseAlignmentPose() {
            return new Pose2d(preciseAlignmentPosition, facing);
        }

        public Command alignmentToBranch(HolonomicDriveSubsystem drive, AprilTagVision aprilTagVision) {
            return Commands.deferredProxy(() -> AutoAlignment.pathFindAndAutoAlign(
                    drive,
                    aprilTagVision,
                    ReefAlignment.getReefAlignmentTarget().roughApproachPose(),
                    ReefAlignment.getReefAlignmentTarget().preciseAlignmentPose(),
                    OptionalInt.of(ReefAlignment.getReefAlignmentTarget().tagId()),
                    Optional.of(FieldMirroringUtils.toCurrentAllianceTranslation(new Translation2d(4.5, 4))),
                    DriveControlLoops.REEF_ALIGNMENT_CONFIG));
        }
    }

    private static int selectedId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static BranchTarget getReefAlignmentTarget() {
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

    public static void setTarget(int targetId) {
        selectedId = targetId;
    }

    public static Command nextTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::nextTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command previousTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::previousTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command followPathAndAlign(RobotContainer robot, PathPlannerPath path, int targetId) {
        return Commands.deferredProxy(() -> {
            BranchTarget branchTarget = (FieldMirroringUtils.isSidePresentedAsRed()
                            ? REEF_ALIGNMENT_POSITIONS_RED
                            : REEF_ALIGNMENT_POSITIONS_BLUE)
                    [targetId];
            return AutoAlignment.followPathAndAutoAlignStatic(
                    robot.drive,
                    robot.aprilTagVision,
                    path,
                    PathUtils.getEndingPose(path),
                    branchTarget.preciseAlignmentPose(),
                    OptionalInt.of(branchTarget.tagId()),
                    DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS);
        });
    }
}
