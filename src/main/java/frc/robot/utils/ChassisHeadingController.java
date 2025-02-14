package frc.robot.utils;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.constants.DriveControlLoops.*;
import static frc.robot.constants.DriveTrainConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Robot;
import frc.robot.constants.DriveControlLoops;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.ironmaple.utils.mathutils.MapleCommonMath;
import org.littletonrobotics.junction.Logger;

/**
 *
 *
 * <h1>Custom Controller for Chassis Heading</h1>
 */
public class ChassisHeadingController {

    /**
     *
     *
     * <h2>Represents an abstract chassis heading request.</h2>
     */
    public abstract static class ChassisHeadingRequest {}

    /**
     *
     *
     * <h2>Represents a request to face a specific rotation.</h2>
     *
     * <p>The chassis is instructed to rotate to and maintain a specified rotation.
     */
    public static class FaceToRotationRequest extends ChassisHeadingRequest {
        public final Rotation2d rotationTarget;

        /** @param rotationTarget the target rotation for the chassis */
        public FaceToRotationRequest(Rotation2d rotationTarget) {
            this.rotationTarget = rotationTarget;
        }
    }

    /**
     *
     *
     * <h2>Represents a request to face a target location on the field.</h2>
     *
     * <p>The chassis is instructed to aim toward a specified target on the field. Optionally, a
     * {@link MapleShooterOptimization} object can be provided to calculate time-of-flight for shooting-on-the-move
     * functionality.
     */
    public static class FaceToTargetRequest extends ChassisHeadingRequest {
        public final Supplier<Translation2d> target;
        public final MapleShooterOptimization shooterOptimization;

        /**
         * @param target the supplier providing the target position on the field
         * @param shooterOptimization optional; used to calculate shooter time-of-flight for shooting-on-the-move
         *     functionality; if not used, pass null
         */
        public FaceToTargetRequest(Supplier<Translation2d> target, MapleShooterOptimization shooterOptimization) {
            this.target = target;
            this.shooterOptimization = shooterOptimization;
        }
    }

    /**
     *
     *
     * <h2>Represents an empty request.</h2>
     *
     * <p>This request will cause {@link #calculate(ChassisSpeeds, Pose2d)} to return zero correction speeds.
     */
    public static class NullRequest extends ChassisHeadingRequest {}

    private final TrapezoidProfile chassisRotationProfile;
    private final MaplePIDController chassisRotationCloseLoop;
    private final double maxAngularVelocityRadPerSec;
    private ChassisHeadingRequest headingRequest;
    private TrapezoidProfile.State chassisRotationState;

    /**
     *
     *
     * <h2>Constructs a heading controller with specific configurations.</h2>
     *
     * @param chassisRotationConstraints defines the maximum angular velocity and acceleration for the drivetrain
     * @param chassisRotationCloseLoopConfig PID configuration for chassis rotation
     * @param chassisInitialFacing the initial orientation of the robot
     */
    public ChassisHeadingController(
            TrapezoidProfile.Constraints chassisRotationConstraints,
            MaplePIDController.MaplePIDConfig chassisRotationCloseLoopConfig,
            Rotation2d chassisInitialFacing) {
        this.chassisRotationProfile = new TrapezoidProfile(chassisRotationConstraints);
        this.chassisRotationCloseLoop = new MaplePIDController(chassisRotationCloseLoopConfig);
        this.headingRequest = new NullRequest();
        this.maxAngularVelocityRadPerSec = chassisRotationConstraints.maxVelocity;
        this.chassisRotationState = new TrapezoidProfile.State(chassisInitialFacing.getRadians(), 0);
    }

    /**
     * Sets a new heading request.
     *
     * @param newRequest the new heading request
     */
    public void setHeadingRequest(ChassisHeadingRequest newRequest) {
        this.headingRequest = newRequest;
    }

    /**
     *
     *
     * <h2>Calculates rotational correction speeds based on the current heading request.</h2>
     *
     * @param measuredSpeedsFieldRelative the measured chassis speeds, field-relative
     * @param robotPose the current pose of the robot as measured by odometry
     * @return (optionally) the calculated correction speed for rotation control, if the request type is not null
     */
    public OptionalDouble calculate(ChassisSpeeds measuredSpeedsFieldRelative, Pose2d robotPose) {
        if (headingRequest instanceof FaceToRotationRequest faceToRotationRequest) {
            return OptionalDouble.of(calculateFaceToRotation(robotPose, faceToRotationRequest.rotationTarget, 0));
        }

        if (headingRequest instanceof FaceToTargetRequest faceToTargetRequest)
            return OptionalDouble.of(calculateFaceToTarget(
                    measuredSpeedsFieldRelative,
                    robotPose,
                    faceToTargetRequest.target.get(),
                    faceToTargetRequest.shooterOptimization));

        chassisRotationState = new TrapezoidProfile.State(
                robotPose.getRotation().getRadians(), measuredSpeedsFieldRelative.omegaRadiansPerSecond);

        log(robotPose, robotPose.getRotation());
        atSetPoint = false;
        return OptionalDouble.empty();
    }

    /**
     *
     *
     * <h2>Calculates rotational correction speeds for a face-to-target request.</h2>
     *
     * <p>For a continuously changing target, feed-forward velocity is added to help the chassis stay aligned. This
     * feed-forward is based on the target's angular velocity relative to the robot, improving tracking accuracy.
     *
     * @param measuredSpeedsFieldRelative the measured chassis speeds, field-relative
     * @param robotPose the current pose of the robot as measured by odometry
     * @param targetPosition the target position to aim at
     * @param shooterOptimization optional {@link MapleShooterOptimization} for shooting-on-the-move functions
     * @return the calculated chassis angular velocity output, in radians/second
     */
    private double calculateFaceToTarget(
            ChassisSpeeds measuredSpeedsFieldRelative,
            Pose2d robotPose,
            Translation2d targetPosition,
            MapleShooterOptimization shooterOptimization) {
        // Target velocity relative to the robot, in the field-origin frame
        final Translation2d targetMovingSpeed = new Translation2d(
                -measuredSpeedsFieldRelative.vxMetersPerSecond, -measuredSpeedsFieldRelative.vyMetersPerSecond);

        final Rotation2d targetedRotation = shooterOptimization == null
                ? targetPosition.minus(robotPose.getTranslation()).getAngle()
                : shooterOptimization.getShooterFacing(
                        targetPosition, robotPose.getTranslation(), measuredSpeedsFieldRelative);
        final Rotation2d targetMovingDirection = targetMovingSpeed.getAngle();
        final Rotation2d positiveRotationTangentDirection =
                targetPosition.minus(robotPose.getTranslation()).getAngle().rotateBy(Rotation2d.fromDegrees(90));

        final double tangentVelocity =
                targetMovingDirection.minus(positiveRotationTangentDirection).getCos() * targetMovingSpeed.getNorm();

        final double distanceToTarget =
                targetPosition.minus(robotPose.getTranslation()).getNorm();

        final double feedforwardAngularVelocity = tangentVelocity / distanceToTarget;

        return calculateFaceToRotation(robotPose, targetedRotation, feedforwardAngularVelocity);
    }

    /**
     *
     *
     * <h2>Calculates rotational correction speeds for a face-to-rotation request.</h2>
     */
    private double calculateFaceToRotation(
            Pose2d robotPose, Rotation2d targetedRotation, double desiredAngularVelocityRadPerSec) {
        TrapezoidProfile.State goalState = getGoalState(targetedRotation);
        chassisRotationState =
                chassisRotationProfile.calculate(Robot.defaultPeriodSecs, chassisRotationState, goalState);

        final double feedBackSpeed =
                chassisRotationCloseLoop.calculate(robotPose.getRotation().getRadians(), chassisRotationState.position);
        final double feedForwardSpeedRadPerSec =
                Math.abs(targetedRotation.minus(robotPose.getRotation()).getDegrees()) < 15
                        ? desiredAngularVelocityRadPerSec
                        : chassisRotationState.velocity;

        log(robotPose, targetedRotation);

        return MapleCommonMath.constrainMagnitude(
                feedBackSpeed + feedForwardSpeedRadPerSec, maxAngularVelocityRadPerSec);
    }

    /**
     *
     *
     * <h2>Determines the closest goal for the targeted rotation.</h2>
     *
     * <p>Finds the closest rotational position on the profile that aligns with the target rotation. This ensures
     * continuity in the rotational profile.
     *
     * @param targetedRotation the desired orientation
     * @return a {@link edu.wpi.first.math.trajectory.TrapezoidProfile.State} representing the target goal state
     */
    private TrapezoidProfile.State getGoalState(Rotation2d targetedRotation) {
        final Rotation2d difference = targetedRotation.minus(Rotation2d.fromRadians(chassisRotationState.position));
        final double goal = chassisRotationState.position + difference.getRadians();
        return new TrapezoidProfile.State(goal, 0);
    }

    private boolean atSetPoint = false;

    private void log(Pose2d robotPose, Rotation2d requestedRotation) {
        Logger.recordOutput(
                "ChassisHeadingController/Requested", new Pose2d(robotPose.getTranslation(), requestedRotation));
        Logger.recordOutput(
                "ChassisHeadingController/CurrentState",
                new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(chassisRotationState.position)));
        final Rotation2d error = requestedRotation.minus(robotPose.getRotation());
        Logger.recordOutput("ChassisHeadingController/Error", error.getDegrees());
        atSetPoint = Math.abs(error.getRadians()) < chassisRotationCloseLoop.getErrorTolerance();
    }

    public boolean atSetPoint() {
        return atSetPoint;
    }

    private static ChassisHeadingController instance = null;

    public static ChassisHeadingController getInstance() {
        if (instance == null)
            instance = new ChassisHeadingController(
                    new TrapezoidProfile.Constraints(
                            ANGULAR_VELOCITY_SOFT_CONSTRAIN.in(RadiansPerSecond),
                            ANGULAR_ACCELERATION_SOFT_CONSTRAIN.in(RadiansPerSecondPerSecond)),
                    DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP,
                    new Rotation2d());

        return instance;
    }
}
