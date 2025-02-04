package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.commands.reefscape.ReefAlignment;

public class ReefConstants {
    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_BLUE = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.01), new Translation2d(3.45, 4.15), 18, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(2.4, 4.01), new Translation2d(3.45, 3.74), 18, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(3.5, 2.3), new Translation2d(3.82, 3.19), 17, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(3.5, 2.3), new Translation2d(4.16, 3.00), 17, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(5.5, 2.3), new Translation2d(4.86, 2.98), 22, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(5.5, 2.3), new Translation2d(5.18, 3.16), 22, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(6.5, 4), new Translation2d(5.51, 3.88), 21, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(6.5, 4), new Translation2d(5.51, 4.17), 21, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(5.08, 4.85), 20, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(5.4, 5.8), new Translation2d(4.93, 5.01), 20, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(4.05, 5.04), 19, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(3.6, 5.8), new Translation2d(3.73, 4.79), 19, true)
    };

    public static final ReefAlignment.BranchTarget[] REEF_ALIGNMENT_POSITIONS_RED = new ReefAlignment.BranchTarget[] {
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(15, 3.8), new Translation2d(14.29, 3.82), 7, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(180), new Translation2d(15, 4.2), new Translation2d(14.28, 4.21), 7, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14.4, 5.9), new Translation2d(13.83, 4.99), 8, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-120), new Translation2d(14, 6.0), new Translation2d(13.49, 5.18), 8, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(12.3, 6.0), new Translation2d(12.61, 5.18), 9, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(-60), new Translation2d(11.8, 5.7), new Translation2d(12.27, 4.98), 9, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 4.3), new Translation2d(11.83, 4.21), 10, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(0), new Translation2d(11.1, 2.8), new Translation2d(11.84, 3.83), 10, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(11.8, 2.3), new Translation2d(12.28, 3.07), 11, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(60), new Translation2d(12.3, 2.1), new Translation2d(12.62, 2.88), 11, true),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(13.9, 2.1), new Translation2d(13.50, 2.86), 6, false),
        new ReefAlignment.BranchTarget(
                Rotation2d.fromDegrees(120), new Translation2d(14.3, 2.3), new Translation2d(13.84, 3.06), 6, true)
    };
}
