package frc.robot.commands.drive;

import static frc.robot.constants.DriveControlLoops.*;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import frc.robot.utils.CustomPIDs.MapleProfiledPIDController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
    private final Supplier<Pose2d> desiredPoseSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final HolonomicDriveController positionController;

    private final double speedConstrainMPS;
    private final Pose2d tolerance;

    public DriveToPose(HolonomicDriveSubsystem driveSubsystem, Supplier<Pose2d> desiredPoseSupplier) {
        this(driveSubsystem, desiredPoseSupplier, new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(10)), 3);
    }

    public DriveToPose(
            HolonomicDriveSubsystem driveSubsystem,
            Supplier<Pose2d> desiredPoseSupplier,
            Pose2d tolerance,
            double speedConstrainMPS) {
        this.desiredPoseSupplier = desiredPoseSupplier;
        this.driveSubsystem = driveSubsystem;
        this.positionController = createPositionController(driveSubsystem);
        this.tolerance = tolerance;
        this.speedConstrainMPS = speedConstrainMPS;

        super.addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        positionController
                .getThetaController()
                .reset(
                        driveSubsystem.getFacing().getRadians(),
                        driveSubsystem.getMeasuredChassisSpeedsRobotRelative().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        ChassisSpeeds feedBackSpeeds = getFeedBackSpeeds();
        final double feedBackSpeedMagnitude =
                Math.hypot(feedBackSpeeds.vxMetersPerSecond, feedBackSpeeds.vyMetersPerSecond);
        if (feedBackSpeedMagnitude < speedConstrainMPS) {
            feedBackSpeeds.vxMetersPerSecond *= speedConstrainMPS / feedBackSpeedMagnitude;
            feedBackSpeeds.vyMetersPerSecond *= speedConstrainMPS / feedBackSpeedMagnitude;
        }
        driveSubsystem.runRobotCentricChassisSpeeds(feedBackSpeeds, false);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }

    /** @return the feed-back speed, robot-relative */
    private ChassisSpeeds getFeedBackSpeeds() {
        Logger.recordOutput(
                "Odometry/TrajectorySetpoint",
                new Pose2d(
                        desiredPoseSupplier.get().getTranslation(),
                        Rotation2d.fromRadians(
                                positionController.getThetaController().getSetpoint().position)));
        return positionController.calculate(
                driveSubsystem.getPose(),
                desiredPoseSupplier.get(),
                0,
                desiredPoseSupplier.get().getRotation());
    }

    @Override
    public boolean isFinished() {
        final Pose2d desiredPose = desiredPoseSupplier.get(), currentPose = driveSubsystem.getPose();
        final ChassisSpeeds speeds = driveSubsystem.getMeasuredChassisSpeedsFieldRelative();
        return Math.abs(desiredPose.getX() - currentPose.getX()) < tolerance.getX()
                && Math.abs(desiredPose.getY() - currentPose.getY()) < tolerance.getY()
                && Math.abs(desiredPose.getRotation().getDegrees()
                                - currentPose.getRotation().getDegrees())
                        < tolerance.getRotation().getDegrees()
                && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) < 0.8
                && Math.abs(speeds.omegaRadiansPerSecond) < Math.toRadians(30);
    }

    public static HolonomicDriveController createPositionController(HolonomicDriveSubsystem driveSubsystem) {
        final TrapezoidProfile.Constraints chassisRotationalConstraints = new TrapezoidProfile.Constraints(
                driveSubsystem.getChassisMaxAngularVelocity(),
                driveSubsystem.getChassisMaxAngularAccelerationRadPerSecSq());
        return new HolonomicDriveController(
                new MaplePIDController(CHASSIS_TRANSLATION_CLOSE_LOOP),
                new MaplePIDController(CHASSIS_TRANSLATION_CLOSE_LOOP),
                new MapleProfiledPIDController(CHASSIS_ROTATION_CLOSE_LOOP, chassisRotationalConstraints));
    }
}
