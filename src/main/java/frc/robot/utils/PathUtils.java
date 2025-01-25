package frc.robot.utils;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.utils.FieldMirroringUtils;

public class PathUtils {
    public static Pose2d getEndingPoseAtBlue(PathPlannerPath path) {
        return new Pose2d(
                path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation(),
                path.getGoalEndState().rotation());
    }

    public static Pose2d getEndingPose(PathPlannerPath path) {
        return FieldMirroringUtils.toCurrentAlliancePose(getEndingPoseAtBlue(path));
    }
}
