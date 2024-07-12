// By 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/ under MIT License

package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.MapleJoystickDriveInput;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.atomic.AtomicReference;

public interface HolonomicDrive extends Subsystem {
    /**
     * runs a ChassisSpeeds without doing any pre-processing
     * @param speeds a discrete chassis speed, robot-centric
     * */
    void runRawChassisSpeeds(ChassisSpeeds speeds);

    /**
     * Returns the current odometry Pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    Pose2d getPose();

    /**
     * Resets the current odometry Pose to a given Pose
     */
    void setPose(Pose2d currentPose);

    /**
     * @return the measured(actual) velocities of the chassis
     * */
    ChassisSpeeds getMeasuredChassisSpeeds();

    double getChassisMaxLinearVelocity();
    double getChassisMaxAngularVelocity();

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp  The timestamp of the vision measurement in seconds.
     */
    void addVisionMeasurement(Pose2d visionPose, double timestamp);

    /**
     * runs a driverstation-centric ChassisSpeeds
     * @param driverStationCentricSpeeds a continuous chassis speeds, driverstation-centric, normally from a gamepad
     * */
    default void runDriverStationCentricChassisSpeeds(ChassisSpeeds driverStationCentricSpeeds) {
        final Rotation2d driverStationFacing = switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> new Rotation2d(Math.PI);
            case Blue -> new Rotation2d(0);
        };
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
        runRawChassisSpeeds(ChassisSpeeds.discretize(speeds, 0.02));
    }

    default void configHolonomicPathPlannerAutoBuilder(double driveBaseRadius) {
        AutoBuilder.configureHolonomic(
                this::getPose,
                this::setPose,
                this::getMeasuredChassisSpeeds,
                this::runRobotCentricChassisSpeeds,
                new HolonomicPathFollowerConfig(getChassisMaxLinearVelocity(), driveBaseRadius, new ReplanningConfig()),
                () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Red),
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]))
        );
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );
    }

    /**
     * @param input the joystick input source
     * @param useFieldCentric whether to do field-centric or not (otherwise, robot-centric)
     * */
    default Command joystickDrive(MapleJoystickDriveInput input, boolean useFieldCentric) {
        final Timer nonSpeedTimer = new Timer();
        final AtomicReference<ChassisSpeeds> currentChassisSpeeds = new AtomicReference<>(new ChassisSpeeds());
        return Commands.run(
                () -> joystickDrivePeriodic(input, useFieldCentric, nonSpeedTimer, currentChassisSpeeds),
                this
        ).beforeStarting(nonSpeedTimer::start);
    }

    default void joystickDrivePeriodic(MapleJoystickDriveInput input, boolean fieldCentricModeOn, Timer nonSpeedTimer, AtomicReference<ChassisSpeeds> currentChassisSpeedsTargetDriverStationCentric) {
        final ChassisSpeeds driveStationCentricSpeed = input.getJoystickChassisSpeeds(
                getChassisMaxLinearVelocity(), getChassisMaxAngularVelocity()
        );
        currentChassisSpeedsTargetDriverStationCentric.set(constrainAcceleration(
                currentChassisSpeedsTargetDriverStationCentric.get(),
                driveStationCentricSpeed,
                getChassisMaxLinearVelocity() / Constants.DriveConfigs.linearAccelerationSmoothOutSeconds,
                getChassisMaxAngularVelocity() / Constants.DriveConfigs.angularAccelerationSmoothOutSeconds,
                Robot.defaultPeriodSecs
        ));
        if (!isZero(driveStationCentricSpeed))
            nonSpeedTimer.reset();

        if (nonSpeedTimer.hasElapsed(Constants.DriveConfigs.nonUsageTimeResetWheels)) {
            stop();
            return;
        }

        if (fieldCentricModeOn)
            runDriverStationCentricChassisSpeeds(currentChassisSpeedsTargetDriverStationCentric.get());
        else
            runRobotCentricChassisSpeeds(currentChassisSpeedsTargetDriverStationCentric.get());
    }

    private boolean isZero(ChassisSpeeds chassisSpeeds) {
        return chassisSpeeds.omegaRadiansPerSecond == 0 && chassisSpeeds.vxMetersPerSecond == 0 && chassisSpeeds.vyMetersPerSecond == 0;
    }

    static ChassisSpeeds constrainAcceleration(
            ChassisSpeeds currentSpeeds, ChassisSpeeds desiredSpeeds,
            double maxLinearAccelerationMetersPerSecSq, double maxAngularAccelerationRadPerSecSq,
            double periodSecs) {
        Translation2d currentLinearVelocityMetersPerSec = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond),
                desiredLinearVelocityMetersPerSec = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond),
                linearVelocityDifference = desiredLinearVelocityMetersPerSec.minus(currentLinearVelocityMetersPerSec);

        final double maxLinearVelocityChangeIn1Period = maxLinearAccelerationMetersPerSecSq * periodSecs;
        final boolean desiredLinearVelocityReachableWithin1Period = linearVelocityDifference.getNorm() <= maxLinearVelocityChangeIn1Period;
        final Translation2d linearVelocityChangeVector = new Translation2d(maxLinearVelocityChangeIn1Period, linearVelocityDifference.getAngle()),
                newLinearVelocity = desiredLinearVelocityReachableWithin1Period ?
                desiredLinearVelocityMetersPerSec
                : currentLinearVelocityMetersPerSec.plus(linearVelocityChangeVector);

        final double angularVelocityDifference = desiredSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond,
                maxAngularVelocityChangeIn1Period = maxAngularAccelerationRadPerSecSq * periodSecs,
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
