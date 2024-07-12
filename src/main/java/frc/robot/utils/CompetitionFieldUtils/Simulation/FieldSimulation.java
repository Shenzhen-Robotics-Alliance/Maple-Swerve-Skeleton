package frc.robot.utils.CompetitionFieldUtils.Simulation;

import frc.robot.utils.CompetitionFieldUtils.MapleCompetitionField;

/**
 * the class that simulates the physical behavior of all the objects on field
 * should only be created during a robot simulation (not in real or replay mode)
 * */
public abstract class FieldSimulation {
    private final MapleCompetitionField competitionField;

    public FieldSimulation(MapleCompetitionField competitionField) {
        this.competitionField = competitionField;
    }
}
