// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/ under MIT License

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.CompetitionFieldUtils.CompetitionFieldVisualizer;
import frc.robot.utils.LocalADStarAK;
import org.littletonrobotics.junction.Logger;

import static frc.robot.constants.JoystickConfigs.*;
import static frc.robot.constants.DriveControlLoops.*;

public interface HolonomicDriveSubsystem extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     * */
    void runRawChassisSpeeds(ChassisSpeeds speeds);

    /**
     * Returns the current odometry Pose.
     */
    Pose2d getPose();

    default Rotation2d getFacing() {return getPose().getRotation(); }

    default Rotation2d getRawGyroYaw() {return getFacing(); }

    /**
     * Resets the current odometry Pose to a given Pose
     */
    void setPose(Pose2d currentPose);

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     * @param measurementStdDevs the standard deviation of the measurement
     */
    default void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> measurementStdDevs) {}

    /**
     * @return the measured(actual) velocities of the chassis, robot-relative
     * */
    ChassisSpeeds getMeasuredChassisSpeedsRobotRelative();

    default ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getMeasuredChassisSpeedsRobotRelative(), getFacing());
    }

    double getChassisMaxLinearVelocityMetersPerSec();
    double getChassisMaxAccelerationMetersPerSecSq();
    double getChassisMaxAngularVelocity();
    double getChassisMaxAngularAccelerationRadPerSecSq();

    default PathConstraints getChassisConstrains(double speedMultiplier) {
        return new PathConstraints(
                getChassisMaxLinearVelocityMetersPerSec() * speedMultiplier,
                getChassisMaxAccelerationMetersPerSecSq() ,
                getChassisMaxAngularVelocity() * speedMultiplier,
                getChassisMaxAngularAccelerationRadPerSecSq()
        );
    }

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     * */
    default void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds) {
        final Rotation2d driverStationFacing = FieldConstants.getDriverStationFacing();
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverStationCentricSpeeds,
                getPose().getRotation().minus(driverStationFacing)
        ));
    }

    /**
     * runs a field-centric ChassisSpeeds
     * @param fieldCentricSpeeds a continuous chassis speeds, field-centric, normally from a pid position controller
     * */
    default void runFieldCentricChassisSpeeds(ChassisSpeeds fieldCentricSpeeds) {
        runRobotCentricChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldCentricSpeeds,
                getPose().getRotation()
        ));
    }

    default void stop() {
        runRobotCentricChassisSpeeds(new ChassisSpeeds());
    }

    /**
     * runs a ChassisSpeeds, pre-processed with ChassisSpeeds.discretize()
     * @param speeds a continuous chassis speed, robot-centric
     * */
    default void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
        final double PERCENT_DEADBAND = 0.03;
        if (Math.abs(speeds.omegaRadiansPerSecond) < PERCENT_DEADBAND * getChassisMaxAngularVelocity()
            && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < PERCENT_DEADBAND * getChassisMaxLinearVelocityMetersPerSec())
            speeds = new ChassisSpeeds();

        runRawChassisSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
    }

    default void configHolonomicPathPlannerAutoBuilder(CompetitionFieldVisualizer fieldVisualizer) {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getMeasuredChassisSpeedsRobotRelative,
                this::runRobotCentricChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        CHASSIS_TRANSLATION_CLOSE_LOOP.toPathPlannerPIDConstants(),
                        CHASSIS_ROTATION_CLOSE_LOOP.toPathPlannerPIDConstants(),
                        getChassisMaxLinearVelocityMetersPerSec(),
                        getChassisMaxLinearVelocityMetersPerSec() / getChassisMaxAngularVelocity(),
                        new ReplanningConfig(false, true)
                ),
                FieldConstants::isSidePresentedAsRed,
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    final Pose2d[] trajectory = activePath.toArray(new Pose2d[0]);
                    Logger.recordOutput("Odometry/Trajectory", trajectory);
                    fieldVisualizer.displayTrajectory(trajectory);
                }
        );
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
    }

    static boolean isZero(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.omegaRadiansPerSecond) < Math.toRadians(5) && Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.05;
    }

    default ChassisSpeeds constrainAcceleration(
            ChassisSpeeds currentSpeeds, ChassisSpeeds desiredSpeeds,
            double dtSecs) {
        final double
                MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ = getChassisMaxLinearVelocityMetersPerSec()
                / LINEAR_ACCELERATION_SMOOTH_OUT_SECONDS,
                MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = getChassisMaxAngularVelocity()
                / ANGULAR_ACCELERATION_SMOOTH_OUT_SECONDS;

        Translation2d currentLinearVelocityMetersPerSec = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                desiredLinearVelocityMetersPerSec = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond),
                linearVelocityDifference = desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec);

        final double maxLinearVelocityChangeIn1Period = MAX_LINEAR_ACCELERATION_METERS_PER_SEC_SQ * dtSecs;
        final boolean desiredLinearVelocityReachableWithin1Period = linearVelocityDifference.getNorm() <= maxLinearVelocityChangeIn1Period;
        final Translation2d linearVelocityChangeVector = new Translation2d(maxLinearVelocityChangeIn1Period, linearVelocityDifference.getAngle()),
                newLinearVelocity = desiredLinearVelocityReachableWithin1Period ?
                desiredLinearVelocityMetersPerSec
                : currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector);

        final double angularVelocityDifference = desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond,
                maxAngularVelocityChangeIn1Period = MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ * dtSecs,
                angularVelocityChange = Math.copySign(maxAngularVelocityChangeIn1Period, angularVelocityDifference);
        final boolean desiredAngularVelocityReachableWithin1Period = Math.abs(angularVelocityDifference) <= maxAngularVelocityChangeIn1Period;
        final double newAngularVelocity = desiredAngularVelocityReachableWithin1Period ?
                desiredSpeeds.omegaRadiansPerSecond
                : currentSpeeds.omegaRadiansPerSecond + angularVelocityChange;
        return new ChassisSpeeds(
                newLinearVelocity.getX(),
                newLinearVelocity.getY(),
                newAngularVelocity
        );
    }
}
