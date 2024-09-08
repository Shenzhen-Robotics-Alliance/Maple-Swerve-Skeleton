package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PathPlannerAuto implements Auto {
    private final String name;
    private final Pose2d robotPlacementPoseAtBlue;

    public PathPlannerAuto(String name, Pose2d robotPlacementPoseAtBlue) {
        this.name = name;
        this.robotPlacementPoseAtBlue = robotPlacementPoseAtBlue;
    }

    @Override
    public Command getAutoCommand(RobotContainer robot) {
        return AutoBuilder.buildAuto(name);
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return robotPlacementPoseAtBlue;
    }
}
