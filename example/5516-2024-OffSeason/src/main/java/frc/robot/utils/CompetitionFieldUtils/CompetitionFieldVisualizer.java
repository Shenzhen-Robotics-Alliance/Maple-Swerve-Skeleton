package frc.robot.utils.CompetitionFieldUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.CompetitionFieldUtils.Objects.GamePieceOnFlyDisplay;
import frc.robot.utils.CompetitionFieldUtils.Objects.RobotOnFieldDisplay;
import org.littletonrobotics.junction.Logger;

import java.util.*;

/**
 * visualizes a competition field on dashboard & Advantage Scope
 * displays robots, opponent robots and game pieces on field
 * the source of these information should either be captured from a vision system during a real competition
 * or by the Maple Physics Simulation during a simulated competition
 * */
public class CompetitionFieldVisualizer {
    public interface ObjectOnFieldDisplay {
        String getTypeName();
        Pose3d getPose3d();
    }

    public interface Object2dOnFieldDisplay extends ObjectOnFieldDisplay {
        Pose2d getObjectOnFieldPose2d();
        @Override
        String getTypeName();
        @Override
        default Pose3d getPose3d() {
            return new Pose3d(getObjectOnFieldPose2d());
        }
    }

    private final Map<String, Set<ObjectOnFieldDisplay>> objectsOnFieldWithGivenType;
    private final Map<String, Set<GamePieceOnFlyDisplay>> gamePiecesOnFlyDisplayWithGivenType;
    public final RobotOnFieldDisplay mainRobot;
    private final Field2d dashboardField2d;
    public CompetitionFieldVisualizer(RobotOnFieldDisplay mainRobot) {
        this.mainRobot = mainRobot;
        this.objectsOnFieldWithGivenType = new HashMap<>();
        this.gamePiecesOnFlyDisplayWithGivenType = new HashMap<>();
        dashboardField2d = new Field2d();
        SmartDashboard.putData("Field", dashboardField2d);
    }

    public ObjectOnFieldDisplay addObject(ObjectOnFieldDisplay object) {
        if (!objectsOnFieldWithGivenType.containsKey(object.getTypeName()))
            objectsOnFieldWithGivenType.put(object.getTypeName(), new HashSet<>());
        objectsOnFieldWithGivenType.get(object.getTypeName()).add(object);
        return object;
    }

    public ObjectOnFieldDisplay deleteObject(ObjectOnFieldDisplay object) {
        if (!objectsOnFieldWithGivenType.containsKey(object.getTypeName()))
            return null;
        if (objectsOnFieldWithGivenType.get(object.getTypeName()).remove(object))
            return object;
        return null;
    }

    public GamePieceOnFlyDisplay addGamePieceOnFly(GamePieceOnFlyDisplay gamePieceOnFlyDisplay) {
        addObject(gamePieceOnFlyDisplay);
        if (!gamePiecesOnFlyDisplayWithGivenType.containsKey(gamePieceOnFlyDisplay.getTypeName()))
            gamePiecesOnFlyDisplayWithGivenType.put(gamePieceOnFlyDisplay.getTypeName(), new HashSet<>());
        gamePiecesOnFlyDisplayWithGivenType.get(gamePieceOnFlyDisplay.getTypeName()).add(gamePieceOnFlyDisplay);
        return gamePieceOnFlyDisplay;
    }

    public Set<ObjectOnFieldDisplay> clearObjectsWithGivenType(String typeName) {
        if (!objectsOnFieldWithGivenType.containsKey(typeName))
            return new HashSet<>();
        final Set<ObjectOnFieldDisplay> originalSet = objectsOnFieldWithGivenType.get(typeName);
        objectsOnFieldWithGivenType.put(typeName, new HashSet<>());
        return originalSet;
    }

    public void displayTrajectory(Pose2d[] trajectory) {
        dashboardField2d.getObject("trajectory").setPoses(trajectory);
    }

    public void updateObjectsToDashboardAndTelemetry() {
        removeGamePiecesOnFlyIfReachedTarget();
        for (String typeName: objectsOnFieldWithGivenType.keySet()) {
            final Set<ObjectOnFieldDisplay> objects = objectsOnFieldWithGivenType.get(typeName);
            Logger.recordOutput("/Field/" + typeName, getPose3ds(objects));
        }

        dashboardField2d.setRobotPose(mainRobot.getObjectOnFieldPose2d());
        Logger.recordOutput("/Field/Robot", mainRobot.getObjectOnFieldPose2d());
    }

    private void removeGamePiecesOnFlyIfReachedTarget() {
        for (Set<GamePieceOnFlyDisplay> gamePieceSet: gamePiecesOnFlyDisplayWithGivenType.values())
            gamePieceSet.removeIf(gamePieceOnFlyDisplay -> {
                if (gamePieceOnFlyDisplay.isReached())
                    deleteObject(gamePieceOnFlyDisplay);
                return gamePieceOnFlyDisplay.isReached();
            });
    }

    private static List<Pose2d> getPose2ds(Set<ObjectOnFieldDisplay> objects) {
        final List<Pose2d> pose2dList = new ArrayList<>();

        for (ObjectOnFieldDisplay object:objects)
            pose2dList.add(object.getPose3d().toPose2d());
        return pose2dList;
    }

    private static Pose3d[] getPose3ds(Set<ObjectOnFieldDisplay> objects) {
        return objects.stream().map(ObjectOnFieldDisplay::getPose3d).toArray(Pose3d[]::new);
    }
}
