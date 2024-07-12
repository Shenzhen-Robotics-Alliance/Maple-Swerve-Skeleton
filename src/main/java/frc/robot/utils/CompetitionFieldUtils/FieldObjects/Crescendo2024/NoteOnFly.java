package frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public class NoteOnFly implements MapleCompetitionField.ObjectOnField {
    private final double launchingTimeStampSec, launchingHeightMeters, launchingSpeedMetersPerSec;
    private final Translation2d positionLaunched;

    public NoteOnFly(double launchingTimeStampSec, double launchingHeightMeters, double launchingSpeedMetersPerSec, Translation2d positionLaunched) {
        this.launchingTimeStampSec = launchingTimeStampSec;
        this.launchingHeightMeters = launchingHeightMeters;
        this.launchingSpeedMetersPerSec = launchingSpeedMetersPerSec;
        this.positionLaunched = positionLaunched;
    }

    @Override
    public String getTypeName() {
        return "Note";
    }

    @Override
    public Pose3d getPose3d() {
        return null; // TODO linear interpret the position when the note is on fly
    }
}
