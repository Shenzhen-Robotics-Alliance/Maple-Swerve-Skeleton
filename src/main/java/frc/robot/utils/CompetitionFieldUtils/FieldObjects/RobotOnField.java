package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

public interface RobotOnField extends MapleCompetitionField.ObjectOn2dField {
    @Override
    default String getTypeName() {
        return "OtherRobots";
    }
}
