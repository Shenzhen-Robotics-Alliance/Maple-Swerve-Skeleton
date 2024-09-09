package frc.robot.utils;

import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.util.PPLibTelemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

public class MaplePathPlannerLoader {
    /**
     * <p> Load a path from a path file in storage </p>
     * <p> Original Source: {@link com.pathplanner.lib.path.PathPlannerPath} </p>
     * <p>
     *  Changes We Made:
     *   1. Supports reversed paths
     *   2. Overrides the global constraints as it different for each robot
     * </p>
     * @param pathName The name of the path to load
     * @return PathPlannerPath created from the given file name
     */
    public static PathPlannerPath fromPathFile(String pathName, PathConstraints globalConstraints) {
        try (BufferedReader br =
                     new BufferedReader(
                             new FileReader(
                                     new File(
                                             Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            PathPlannerPath path = pathFromJson(json, globalConstraints);
            PPLibTelemetry.registerHotReloadPath(pathName, path);
            return path;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * <p> Load a path from a path file in storage </p>
     * <p> Original Source: {@link com.pathplanner.lib.path.PathPlannerPath} </p>
     * <p>
     *  Changes We Made:
     *   1. Supports reversed paths
     *   2. Overrides the global constraints as it different for each robot
     * </p>
     * @param pathName The name of the path to load
     * @return PathPlannerPath created from the given file name
     */
    public static PathPlannerPath fromPathFileReversed(String pathName, PathConstraints globalConstraints, GoalEndState goalEndState) {
        try (BufferedReader br =
                     new BufferedReader(
                             new FileReader(
                                     new File(
                                             Filesystem.getDeployDirectory(), "pathplanner/paths/" + pathName + ".path")))) {
            StringBuilder fileContentBuilder = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) {
                fileContentBuilder.append(line);
            }

            String fileContent = fileContentBuilder.toString();
            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);

            PathPlannerPath path = pathReversedFromJson(json, globalConstraints, goalEndState);
            PPLibTelemetry.registerHotReloadPath(pathName, path);
            return path;
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }


    private static PathPlannerPath pathFromJson(JSONObject pathJson, PathConstraints globalConstraints) {
        GoalEndState goalEndState = goalEndStateFromJson((JSONObject) pathJson.get("goalEndState"));
        return pathFromJson(pathJson, globalConstraints, false, goalEndState);
    }

    private static PathPlannerPath pathReversedFromJson(JSONObject pathJson, PathConstraints globalConstraints, GoalEndState goalEndState) {
        return pathFromJson(pathJson, globalConstraints, true, goalEndState);
    }

    /**
     * Original Source: {@link com.pathplanner.lib.path.PathPlannerPath}
     * Changes We Made:
     * 1. Supports reversed paths
     * 2. Overrides the global constraints as it different for each robot
     *
     * @param pathJson
     * @param globalConstraints
     * @param reversed
     * */
    private static PathPlannerPath pathFromJson(JSONObject pathJson, PathConstraints globalConstraints, boolean reversed, GoalEndState goalEndState) {
        final JSONArray wayPointsJsonArray = (JSONArray) pathJson.get("waypoints");
        final int wayPointsCount = wayPointsJsonArray.size();
        List<Translation2d> bezierPoints = bezierPointsFromWaypointsJson(wayPointsJsonArray, reversed);
        List<RotationTarget> rotationTargets = new ArrayList<>();
        List<ConstraintsZone> constraintZones = new ArrayList<>();
        List<EventMarker> eventMarkers = new ArrayList<>();

        for (var rotJson : (JSONArray) pathJson.get("rotationTargets")) {
            final RotationTarget originalRotationTarget = rotationTargetFromJson((JSONObject) rotJson);
            final double wayPointRelativePosition = reversed ?
                    wayPointsCount - originalRotationTarget.getPosition()
                    : originalRotationTarget.getPosition();
            rotationTargets.add(new RotationTarget(
                    wayPointRelativePosition,
                    originalRotationTarget.getTarget(),
                    originalRotationTarget.shouldRotateFast()
            ));
        }

        for (var zoneJson : (JSONArray) pathJson.get("constraintZones")) {
            final ConstraintsZone originalConstraintsZone = constraintsZoneFromJson((JSONObject) zoneJson);
            final double minWayPointPos = reversed ?
                    wayPointsCount - originalConstraintsZone.getMaxWaypointPos()
                    : originalConstraintsZone.getMinWaypointPos();
            final double maxWayPointPos = reversed ?
                    wayPointsCount - originalConstraintsZone.getMinWaypointPos()
                    : originalConstraintsZone.getMaxWaypointPos();
            constraintZones.add(
                    new ConstraintsZone(minWayPointPos, maxWayPointPos, originalConstraintsZone.getConstraints())
            );
        }

        for (var markerJson : (JSONArray) pathJson.get("eventMarkers")) {
            final EventMarker originalEventMarker = eventMarkerFromJson((JSONObject) markerJson);
            final double wayPointRelativePosition = reversed ?
                    wayPointsCount - originalEventMarker.getWaypointRelativePos()
                    : originalEventMarker.getWaypointRelativePos();
            eventMarkers.add(new EventMarker(wayPointRelativePosition, originalEventMarker.getCommand()));
        }

        Rotation2d previewStartingRotation = Rotation2d.fromDegrees(0);
        if (pathJson.containsKey("previewStartingState")) {
            JSONObject previewStartingStateJson = (JSONObject) pathJson.get("previewStartingState");
            if (previewStartingStateJson != null) {
                previewStartingRotation =
                        Rotation2d.fromDegrees(
                                ((Number) previewStartingStateJson.get("rotation")).doubleValue());
            }
        }

        return new PathPlannerPath(
                bezierPoints,
                rotationTargets,
                constraintZones,
                eventMarkers,
                globalConstraints,
                goalEndState,
                false,
                previewStartingRotation);
    }

    /**
     * original source: {@link com.pathplanner.lib.path.PathPlannerPath}
     *
     * added path reversing function
     * */
    private static List<Translation2d> bezierPointsFromWaypointsJson(JSONArray waypointsJson, boolean reversed) {
        List<Translation2d> bezierPoints = new ArrayList<>();

        // First point
        JSONObject firstPoint = (JSONObject) waypointsJson.get(0);
        bezierPoints.add(pointFromJson((JSONObject) firstPoint.get("anchor")));
        bezierPoints.add(pointFromJson((JSONObject) firstPoint.get("nextControl")));

        // Mid points
        for (int i = 1; i < waypointsJson.size() - 1; i++) {
            JSONObject point = (JSONObject) waypointsJson.get(i);
            bezierPoints.add(pointFromJson((JSONObject) point.get("prevControl")));
            bezierPoints.add(pointFromJson((JSONObject) point.get("anchor")));
            bezierPoints.add(pointFromJson((JSONObject) point.get("nextControl")));
        }

        // Last point
        JSONObject lastPoint = (JSONObject) waypointsJson.get(waypointsJson.size() - 1);
        bezierPoints.add(pointFromJson((JSONObject) lastPoint.get("prevControl")));
        bezierPoints.add(pointFromJson((JSONObject) lastPoint.get("anchor")));

        return reversed ? revereList(bezierPoints) : bezierPoints;
    }

    private static  <E> List<E> revereList(List<E> originalList) {
        final ArrayList<E> reversedList = new ArrayList<>();
        for (int i = originalList.size()-1; i >= 0; i--)
            reversedList.add(originalList.get(i));
        return reversedList;
    }

    /**
     * original source:  {@link com.pathplanner.lib.path.PathPlannerPath}
     * */
    private static Translation2d pointFromJson(JSONObject pointJson) {
        double x = ((Number) pointJson.get("x")).doubleValue();
        double y = ((Number) pointJson.get("y")).doubleValue();

        return new Translation2d(x, y);
    }

    /**
     * Original Source {@link com.pathplanner.lib.path.GoalEndState}
     * Create a goal end state from json
     *
     * @param endStateJson {@link org.json.simple.JSONObject} representing a goal end state
     * @return The goal end state defined by the given json
     */
    private static GoalEndState goalEndStateFromJson(JSONObject endStateJson) {
        double vel = ((Number) endStateJson.get("velocity")).doubleValue();
        double deg = ((Number) endStateJson.get("rotation")).doubleValue();
        boolean rotateFast = false;
        if (endStateJson.get("rotateFast") != null) {
            rotateFast = (boolean) endStateJson.get("rotateFast");
        }
        return new GoalEndState(vel, Rotation2d.fromDegrees(deg), rotateFast);
    }

    /**
     * Create a rotation target from json
     *
     * @param targetJson {@link org.json.simple.JSONObject} representing a rotation target
     * @return Rotation target defined by the given json
     */
    private static RotationTarget rotationTargetFromJson(JSONObject targetJson) {
        double pos = ((Number) targetJson.get("waypointRelativePos")).doubleValue();
        double deg = ((Number) targetJson.get("rotationDegrees")).doubleValue();
        boolean rotateFast = false;
        if (targetJson.get("rotateFast") != null) {
            rotateFast = (boolean) targetJson.get("rotateFast");
        }
        return new RotationTarget(pos, Rotation2d.fromDegrees(deg), rotateFast);
    }

    /**
     * Original Source {@link com.pathplanner.lib.path.ConstraintsZone}
     * Create a constraints zone from json
     *
     * @param zoneJson A {@link org.json.simple.JSONObject} representing a constraints zone
     * @return The constraints zone defined by the given json object
     */
    private static ConstraintsZone constraintsZoneFromJson(JSONObject zoneJson) {
        double minPos = ((Number) zoneJson.get("minWaypointRelativePos")).doubleValue();
        double maxPos = ((Number) zoneJson.get("maxWaypointRelativePos")).doubleValue();
        PathConstraints constraints =
                pathConstraintsFromJson((JSONObject) zoneJson.get("constraints"));
        return new ConstraintsZone(minPos, maxPos, constraints);
    }

    /**
     * Original Source {@link com.pathplanner.lib.path.PathConstraints}
     * Create a path constraints object from json
     *
     * @param constraintsJson {@link org.json.simple.JSONObject} representing a path constraints
     *     object
     * @return The path constraints defined by the given json
     */
    private static PathConstraints pathConstraintsFromJson(JSONObject constraintsJson) {
        double maxVel = ((Number) constraintsJson.get("maxVelocity")).doubleValue();
        double maxAccel = ((Number) constraintsJson.get("maxAcceleration")).doubleValue();
        double maxAngularVel =
                ((Number) constraintsJson.get("maxAngularVelocity")).doubleValue(); // Degrees
        double maxAngularAccel =
                ((Number) constraintsJson.get("maxAngularAcceleration")).doubleValue(); // Degrees

        return new PathConstraints(
                maxVel,
                maxAccel,
                Units.degreesToRadians(maxAngularVel),
                Units.degreesToRadians(maxAngularAccel));
    }

    /**
     * Original Source {@link com.pathplanner.lib.path.EventMarker}
     * Create an event marker from json
     *
     * @param markerJson {@link org.json.simple.JSONObject} representing an event marker
     * @return The event marker defined by the given json object
     */
    private static EventMarker eventMarkerFromJson(JSONObject markerJson) {
        double pos = ((Number) markerJson.get("waypointRelativePos")).doubleValue();
        Command cmd = CommandUtil.commandFromJson((JSONObject) markerJson.get("command"), false);
        return new EventMarker(pos, cmd);
    }

    public static Supplier<Pose2d> getEndingRobotPoseInCurrentAllianceSupplier(PathPlannerPath pathAtBlueAlliance) {
        final List<Pose2d> pathPoses =  pathAtBlueAlliance.getPathPoses();
        final Rotation2d endingRotation = pathAtBlueAlliance.getGoalEndState().getRotation();
        final Pose2d lastPathPose = pathPoses.get(pathPoses.size()-1);

        return () -> FieldConstants.toCurrentAlliancePose(new Pose2d(lastPathPose.getTranslation(), endingRotation));
    }
}