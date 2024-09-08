package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ExampleAuto implements Auto {
    @Override
    public Command getAutoCommand(RobotContainer robot) {
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"));
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return new Pose2d(1.3, 7.2, Rotation2d.fromDegrees(180));
    }
}
