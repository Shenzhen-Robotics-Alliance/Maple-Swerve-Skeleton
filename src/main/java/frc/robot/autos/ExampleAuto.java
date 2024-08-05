package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drive.FollowPathPP;
import frc.robot.utils.MaplePathPlannerLoader;

public class ExampleAuto extends Auto {
    private final Pose2d startingPose;
    public ExampleAuto(RobotContainer robot) {
        super();
        super.setName("Example Auto");
        super.addRequirements(robot.drive);

        final PathPlannerPath testPath = MaplePathPlannerLoader.fromPathFile(
                "Test Path", robot.drive.getChassisConstrains(1)
        );
        this.startingPose = testPath.getPreviewStartingHolonomicPose();
        super.addCommands(new FollowPathPP(testPath, Constants::isSidePresentedAsRed, robot.drive));
    }

    @Override
    public Pose2d getStartingPoseAtBlueAlliance() {
        return startingPose;
    }
}
