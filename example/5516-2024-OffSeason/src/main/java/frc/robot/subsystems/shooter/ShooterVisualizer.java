package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.LogPaths;
import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.PitchConstants.*;

public class ShooterVisualizer {
    public static final Translation3d
            /* the translation of the shooter to robot, in the robot's frame */
            SHOOTER_TRANSLATION_ON_ROBOT = new Translation3d(-0.25, 0.24, 0.09),
            /* the translation of the note to the robot, when the center of the note is aligned with the pitch axle */
            NOTE_AT_SHOOTER_AXLE_TRANSLATION_ON_ROBOT = new Translation3d(-0.25, 0 , 0.1),
            /*  */
            NOTE_TRANSLATION_AT_PITCH_DIRECTION_TO_AXLE = new Translation3d(0.2, 0 , 0);

    public static double pitchAngleRad = PITCH_LOWEST_ROTATION_RAD;
    public enum NotePositionInShooter {
        GONE,
        AT_BOTTOM,
        AT_TOP
    }
    public static NotePositionInShooter notePositionInShooter = NotePositionInShooter.GONE;

    public static void setPitchAngle(double pitchAngleRad) {
        ShooterVisualizer.pitchAngleRad = pitchAngleRad;
    }

    public static void setNoteInShooter(NotePositionInShooter notePositionInShooter) {
        ShooterVisualizer.notePositionInShooter = notePositionInShooter;
    }

    public static void showResultsToDashboard(Pose3d robotPose) {
        final Pose3d shooterPoseToRobot = new Pose3d(
                SHOOTER_TRANSLATION_ON_ROBOT,
                new Rotation3d(0, -pitchAngleRad, 0));
        Logger.recordOutput(LogPaths.SHOOTER_PATH + "MechanismPose", shooterPoseToRobot);
        final Transform3d robotToNote = new Transform3d(
                NOTE_AT_SHOOTER_AXLE_TRANSLATION_ON_ROBOT,
                new Rotation3d(0, -pitchAngleRad, 0)
        );
        final Transform3d noteTranslationInPitchDirection = new Transform3d(
                switch (notePositionInShooter) {
                    case GONE, AT_BOTTOM -> new Translation3d();
                    case AT_TOP -> NOTE_TRANSLATION_AT_PITCH_DIRECTION_TO_AXLE;
                },
                new Rotation3d()
        );
        final Pose3d notePose = switch (notePositionInShooter) {
            case AT_TOP, AT_BOTTOM -> robotPose.plus(robotToNote).plus(noteTranslationInPitchDirection);
            case GONE -> new Pose3d(0, 0, -1, new Rotation3d());
        };
        Logger.recordOutput(LogPaths.SHOOTER_PATH + "NoteInShooter", notePose);
    }
}
