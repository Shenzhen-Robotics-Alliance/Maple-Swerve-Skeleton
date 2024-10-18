package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PathPlannerAutoWrapper implements Auto {
    private final Pose2d startingPose;
    private final String name;

    public PathPlannerAutoWrapper(String name) {
        this.name = name;
        this.startingPose = new PathPlannerAuto(name).getStartingPose();
    }

    @Override
    public Command getAutoCommand(RobotContainer robot) {
        return AutoBuilder.buildAuto(name);
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return startingPose;
    }
}
