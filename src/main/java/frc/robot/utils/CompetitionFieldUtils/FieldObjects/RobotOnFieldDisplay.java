package frc.robot.utils.CompetitionFieldUtils.FieldObjects;

import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

/**
 * displays a robot on field
 * note that the main robot also inherits this class,
 * but it will not be displayed in "Robots" Pose3d array
 * it will be displayed as "Robot" and with a single Pose2d
 * */
public interface RobotOnFieldDisplay extends MapleCompetitionField.Object2dOnFieldDisplay {
    @Override
    default String getTypeName() {return "Robots";}
}
