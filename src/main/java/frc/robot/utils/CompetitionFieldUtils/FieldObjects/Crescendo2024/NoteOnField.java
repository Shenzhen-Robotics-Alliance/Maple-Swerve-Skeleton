package frc.robot.utils.CompetitionFieldUtils.FieldObjects.Crescendo2024;

import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public interface NoteOnField extends MapleCompetitionField.ObjectOn2dField {
    @Override
    default String getTypeName() {
        return "Note";
    }
}
