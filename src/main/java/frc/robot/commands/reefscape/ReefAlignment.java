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
import org.littletonrobotics.junction.Logger;

public class ReefAlignment {
    private static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.5, 4);

    public record BranchTarget(
            Rotation2d facing, Translation2d roughApproachPosition, Translation2d preciseAlignmentPosition, int tagId) {

        public Pose2d roughApproachPose() {
            return new Pose2d(roughApproachPosition, facing);
        }

        public Pose2d preciseAlignmentPose() {
            return new Pose2d(preciseAlignmentPosition, facing);
        }

        public int cameraToFocus() {
            return tagId % 2 == 0
                    ? 1 // lefter targets uses righter camera
                    : 0; // righter targets uses lefter camera
        }
    }

    // 0 to 5
    private static int selectedReefPartId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static BranchTarget getReefAlignmentTarget(boolean rightSide) {
        int branchIndex = getBranchIndexFromReefPartId(rightSide);
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[branchIndex]
                : REEF_ALIGNMENT_POSITIONS_BLUE[branchIndex];
    }

    private static int getBranchIndexFromReefPartId(boolean rightSide) {
        int branchIndex = selectedReefPartId * 2;
        boolean isUpperSide = // selectedReefPartId == 2 || selectedReefPartId == 3 || selectedReefPartId == 4;
                false;
        if (rightSide ^ isUpperSide) branchIndex++;
        return branchIndex;
    }

    public static boolean[] displaySelectedBranch() {
        boolean[] reef = new boolean[12];
        Arrays.fill(reef, false);
        switch (selectedSide) {
            case LEFT -> reef[getBranchIndexFromReefPartId(false)] = true;
            case RIGHT -> reef[getBranchIndexFromReefPartId(true)] = true;
            case NOT_SELECTED -> reef[getBranchIndexFromReefPartId(false)] =
                    reef[getBranchIndexFromReefPartId(true)] = true;
        }
        return reef;
    }

    public static void nextTarget() {
        if (++selectedReefPartId >= 6) selectedReefPartId = 0;
    }

    public static void previousTarget() {
        if (--selectedReefPartId < 0) selectedReefPartId = 5;
    }

    public static void selectReefPart(int reefPartId) {
        selectedReefPartId = reefPartId;
    }

    public static Command selectReefPartButton(int reefPartId) {
        return Commands.runOnce(() -> selectReefPart(reefPartId), lock);
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

    public static void lefterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) previousTarget();
        else nextTarget();
    }

    public static void righterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) nextTarget();
        else previousTarget();
    }

    public static Command lefterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::lefterTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command righterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::righterTarget, lock)
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
                            branchTarget.facing(),
                            OptionalInt.of(branchTarget.tagId()),
                            OptionalInt.of(branchTarget.cameraToFocus()),
                            DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS)
                    .beforeStarting(() -> {
                        selectedReefPartId = targetId / 2;
                        selectedSide = targetId % 2 == 0 ? SelectedSide.LEFT : SelectedSide.RIGHT;
                    })
                    .finallyDo(() -> selectedSide = SelectedSide.NOT_SELECTED);
        });
    }

    public static Command alignmentToBranch(
            HolonomicDriveSubsystem drive, AprilTagVision aprilTagVision, boolean rightSide) {
        return Commands.deferredProxy(() -> AutoAlignment.pathFindAndAutoAlign(
                        drive,
                        aprilTagVision,
                        ReefAlignment.getReefAlignmentTarget(rightSide).roughApproachPose(),
                        ReefAlignment.getReefAlignmentTarget(rightSide).preciseAlignmentPose(),
                        ReefAlignment.getReefAlignmentTarget(rightSide).facing(),
                        OptionalInt.of(
                                ReefAlignment.getReefAlignmentTarget(rightSide).tagId()),
                        OptionalInt.of(rightSide ? 0 : 1), // right side uses lefter cam
                        Optional.of(FieldMirroringUtils.toCurrentAllianceTranslation(REEF_CENTER_BLUE)),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG))
                .beforeStarting(() -> selectedSide = rightSide ? SelectedSide.RIGHT : SelectedSide.LEFT)
                .finallyDo(() -> selectedSide = SelectedSide.NOT_SELECTED);
    }

    private enum SelectedSide {
        LEFT,
        RIGHT,
        NOT_SELECTED
    }

    private static SelectedSide selectedSide = SelectedSide.NOT_SELECTED;

    public static void updateDashboard() {
        Logger.recordOutput("Reef/SelectedBranch", ReefAlignment.displaySelectedBranch());
    }
}
