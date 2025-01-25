package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.reefscape.ReefAlignment;

public class ReefConstants {
    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(6.6, 4.01), new Translation2d(3.45, 4.15), 18),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(6.6, 4.01), new Translation2d(3.45, 3.74), 18),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(3.5, 2.3), new Translation2d(3.82, 3.19), 17),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(3.5, 2.3), new Translation2d(4.16, 3.00), 17),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(5.5, 2.3), new Translation2d(4.86, 2.98), 22),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(5.5, 2.3), new Translation2d(5.18, 3.16), 22),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(6.5, 4), new Translation2d(5.51, 3.88), 21),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(6.5, 4), new Translation2d(5.51, 4.17), 21),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(5.08, 4.85), 20),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(4.93, 5.01), 20),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(4.05, 5.04), 19),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.73, 4.79), 19)
    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(11.0, 4.09), new Translation2d(14.1, 3.9), 7),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(11.0, 4.09), new Translation2d(14.1, 4.31), 7),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14.1, 5.8), new Translation2d(13.73, 4.86), 8),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14.1, 5.8), new Translation2d(13.39, 5.05), 8),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(12.1, 5.8), new Translation2d(12.69, 5.07), 9),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(12.1, 5.8), new Translation2d(12.37, 4.89), 9),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 4.1), new Translation2d(12.04, 4.17), 10),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 4.1), new Translation2d(12.04, 3.88), 10),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(12.2, 2.3), new Translation2d(12.47, 3.20), 11),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(12.2, 2.3), new Translation2d(12.62, 3.04), 11),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(14.0, 2.3), new Translation2d(13.50, 3.01), 6),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(14.0, 2.3), new Translation2d(13.82, 3.26), 6)
    };
}
