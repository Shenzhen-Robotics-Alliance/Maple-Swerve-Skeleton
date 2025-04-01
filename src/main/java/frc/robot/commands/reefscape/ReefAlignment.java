package frc.robot.commands.reefscape;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.constants.DriveControlLoops.*;
import static frc.robot.constants.ReefConstants.*;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.drive.AutoAlignment;
import frc.robot.constants.DriveControlLoops;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

public class ReefAlignment {
    public static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.5, 4);

    public enum Side {
        LEFT,
        RIGHT,
        CENTER
    }

    public record BranchTarget(
            Rotation2d facing,
            Translation2d roughApproachPosition,
            Translation2d preciseAlignmentPosition,
            int tagId,
            Side side) {

        public Pose2d roughApproachPose() {
            return new Pose2d(roughApproachPosition, facing);
        }

        public Pose2d preciseAlignmentPose() {
            return new Pose2d(preciseAlignmentPosition, facing);
        }

        public AutoAlignment.AutoAlignmentTarget autoAlignmentTarget() {
            return new AutoAlignment.AutoAlignmentTarget(
                    roughApproachPose(),
                    preciseAlignmentPose(),
                    facing(),
                    Optional.of(FieldMirroringUtils.toCurrentAllianceTranslation(REEF_CENTER_BLUE)),
                    OptionalInt.of(tagId),
                    switch (side) {
                        case LEFT -> new Integer[] {1, 3};
                        case RIGHT -> new Integer[] {0, 2};
                        case CENTER -> new Integer[] {};
                    });
        }

        public static BranchTarget measured(int tagID, Side side, Distance distanceToTag, Distance biasFromCenter) {
            Pose2d tagPose =
                    VisionConstants.fieldLayout.getTagPose(tagID).orElseThrow().toPose2d();
            double biasFromCenterFactor =
                    switch (side) {
                        case LEFT -> -1.0;
                        case RIGHT -> 1.0;
                        case CENTER -> 0.0;
                    };
            Twist2d tagToTarget =
                    new Twist2d(distanceToTag.in(Meters), biasFromCenterFactor * biasFromCenter.in(Meters), 0);
            Twist2d tagToRoughTarget = new Twist2d(
                    ROUGH_APPROACHT_POSE_TO_TARGET_DISTANCE.in(Meters),
                    biasFromCenterFactor * ROUGH_APPROACH_POSE_TO_TARGET_MARGIN.in(Meters),
                    0);
            Translation2d targetPosition = tagPose.exp(tagToTarget).getTranslation();
            Translation2d roughTargetPosition = tagPose.exp(tagToRoughTarget).getTranslation();

            return new BranchTarget(
                    tagPose.getRotation().rotateBy(Rotation2d.k180deg),
                    roughTargetPosition,
                    targetPosition,
                    tagID,
                    side);
        }
    }

    // 0 to 5
    private static int selectedReefPartId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static BranchTarget getSelectedReefAlignmentTarget(Side side) {
        int branchIndex = getBranchIndexFromReefPartId(selectedReefPartId, side);
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[branchIndex]
                : REEF_ALIGNMENT_POSITIONS_BLUE[branchIndex];
    }

    public static BranchTarget getNearestReefAlignmentTarget(Translation2d robotPosition, Side side) {
        int index = getNearestReefAlignmentTargetId(robotPosition, side);
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[index]
                : REEF_ALIGNMENT_POSITIONS_BLUE[index];
    }

    public static int getNearestReefAlignmentTargetId(Translation2d robotPosition, Side side) {
        int minDistanceTargetId = -1;
        double minDistance = Double.POSITIVE_INFINITY;
        for (int i = 0; i < REEF_ALIGNMENT_POSITIONS_BLUE.length; i++) {
            BranchTarget target = FieldMirroringUtils.isSidePresentedAsRed()
                    ? REEF_ALIGNMENT_POSITIONS_RED[i]
                    : REEF_ALIGNMENT_POSITIONS_BLUE[i];
            Pose3d tagPose = VisionConstants.fieldLayout
                    .getTagPose(target.tagId)
                    .orElse(new Pose3d(0, 0, -100, new Rotation3d()));
            double robotToTargetDistance =
                    tagPose.toPose2d().getTranslation().minus(robotPosition).getNorm();
            if (robotToTargetDistance > minDistance || side != target.side) continue;
            minDistance = robotToTargetDistance;
            minDistanceTargetId = i;
        }
        return minDistanceTargetId;
    }

    private static int getBranchIndexFromReefPartId(int reefPartId, Side side) {
        int branchIndex = reefPartId * 2;
        branchIndex += switch (side) {
            case LEFT -> 0;
            case RIGHT -> 1;
            case CENTER -> 24;};
        return branchIndex;
    }

    private static boolean[] reef = new boolean[12];

    public static boolean[] displaySelectedBranch() {
        Arrays.fill(reef, false);
        if (selectedSide == Side.CENTER || selectedSide == Side.LEFT)
            reef[getBranchIndexFromReefPartId(selectedReefPartId, Side.LEFT)] = true;
        if (selectedSide == Side.CENTER || selectedSide == Side.RIGHT)
            reef[getBranchIndexFromReefPartId(selectedReefPartId, Side.RIGHT)] = true;
        return reef;
    }

    public static boolean[] displayNearestBranch() {
        Arrays.fill(reef, false);
        Translation2d robotPosition = RobotState.getInstance().getVisionPose().getTranslation();
        if (selectedSide == Side.CENTER || selectedSide == Side.LEFT)
            reef[getNearestReefAlignmentTargetId(robotPosition, Side.LEFT)] = true;
        if (selectedSide == Side.CENTER || selectedSide == Side.RIGHT)
            reef[getNearestReefAlignmentTargetId(robotPosition, Side.RIGHT)] = true;
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

    public static Command followPathAndAlign(
            RobotContainer robot, PathPlannerPath path, int targetId, Command... toScheduleBeforePreciseAlignment) {
        return Commands.deferredProxy(() -> {
            BranchTarget branchTarget = (FieldMirroringUtils.isSidePresentedAsRed()
                            ? REEF_ALIGNMENT_POSITIONS_RED
                            : REEF_ALIGNMENT_POSITIONS_BLUE)
                    [targetId];
            return AutoAlignment.followPathAndAutoAlignStatic(
                            robot.drive,
                            robot.aprilTagVision,
                            robot.ledStatusLight,
                            path,
                            branchTarget.autoAlignmentTarget(),
                            DriveControlLoops.REEF_ALIGNMENT_CONFIG_AUTONOMOUS,
                            toScheduleBeforePreciseAlignment)
                    .beforeStarting(() -> {
                        selectedReefPartId = targetId / 2;
                        selectedSide = targetId % 2 == 0 ? Side.LEFT : Side.RIGHT;
                    })
                    .finallyDo(() -> selectedSide = Side.CENTER);
        });
    }

    public static Command alignmentToSelectedBranch(
            HolonomicDriveSubsystem drive,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            Side side,
            AutoAlignment.AutoAlignmentConfigurations config,
            Command... toScheduleAtPreciseAlignment) {
        return Commands.deferredProxy(() -> pathFindAndAlignToBranchStatic(
                        drive,
                        aprilTagVision,
                        statusLight,
                        getSelectedReefAlignmentTarget(side),
                        config,
                        toScheduleAtPreciseAlignment))
                .withName("[Reef Alignment] Align to branch " + getBranchIndexFromReefPartId(selectedReefPartId, side))
                .beforeStarting(() -> selectedSide = side)
                .finallyDo(() -> selectedSide = Side.CENTER);
    }

    public static Command alignToNearestBranch(
            HolonomicDriveSubsystem drive,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            Side side,
            AutoAlignment.AutoAlignmentConfigurations config,
            Command... toScheduleAtPreciseAlignment) {
        return Commands.deferredProxy(() -> pathFindAndAlignToBranchStatic(
                drive,
                aprilTagVision,
                statusLight,
                getNearestReefAlignmentTarget(
                        RobotState.getInstance().getVisionPose().getTranslation(), side),
                config,
                toScheduleAtPreciseAlignment));
    }

    private static Command pathFindAndAlignToBranchStatic(
            HolonomicDriveSubsystem drive,
            AprilTagVision aprilTagVision,
            LEDStatusLight statusLight,
            BranchTarget branch,
            AutoAlignment.AutoAlignmentConfigurations config,
            Command... toScheduleAtPreciseAlignment) {
        return AutoAlignment.pathFindAndAutoAlignStatic(
                        drive,
                        aprilTagVision,
                        statusLight,
                        branch.autoAlignmentTarget(),
                        config,
                        toScheduleAtPreciseAlignment)
                .withName("[Reef Alignment] Align to branch");
    }

    private static Side selectedSide = Side.CENTER;

    public static void updateDashboard() {
        Logger.recordOutput("Reef/SelectedBranch", ReefAlignment.displaySelectedBranch());
        Logger.recordOutput("Reef/NearestBranch", ReefAlignment.displayNearestBranch());
        int nearestBranchTagID = getNearestReefAlignmentTarget(
                        RobotState.getInstance().getVisionPose().getTranslation(), Side.CENTER)
                .tagId;
        Optional<Pose3d> tagPose3d = VisionConstants.fieldLayout.getTagPose(nearestBranchTagID);
        if (tagPose3d.isEmpty()) return;
        Pose2d tagRawPose = tagPose3d.get().toPose2d();
        Pose2d tagPose =
                new Pose2d(tagRawPose.getTranslation(), tagRawPose.getRotation().rotateBy(Rotation2d.k180deg));
        Logger.recordOutput(
                "Reef/RobotToNearestBranchTag",
                RobotState.getInstance().getVisionPose().log(tagPose));
    }
}
