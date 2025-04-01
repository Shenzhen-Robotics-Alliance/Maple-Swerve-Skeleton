package frc.robot.subsystems.drive;

import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.utils.CustomPIDs.MaplePIDController;

public class MapleHolonomicDriveController implements PathFollowingController {
    private final MaplePIDController xController;
    private final MaplePIDController yCOntroller;
    private final MaplePIDController rotationController;

    public MapleHolonomicDriveController(
            MaplePIDController.MaplePIDConfig translationalPIDConfig,
            MaplePIDController.MaplePIDConfig rotationalPIDConfig) {
        xController = new MaplePIDController(translationalPIDConfig);
        yCOntroller = new MaplePIDController(translationalPIDConfig);
        rotationController = new MaplePIDController(rotationalPIDConfig);

        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        Translation2d translationalFeedforwardSpeeds =
                new Translation2d(targetState.fieldSpeeds.vxMetersPerSecond, targetState.fieldSpeeds.vyMetersPerSecond);
        Translation2d translationalFeedbackSpeeds = new Translation2d(
                this.xController.calculate(currentPose.getX(), targetState.pose.getX()),
                this.yCOntroller.calculate(currentPose.getY(), targetState.pose.getY()));
        Translation2d translationalOutputSpeeds = translationalFeedforwardSpeeds.plus(translationalFeedbackSpeeds);

        double rotationalFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetState.pose.getRotation().getRadians());
        double rotationalFeedforward = targetState.fieldSpeeds.omegaRadiansPerSecond;

        ChassisSpeeds fieldRelativeOutput = new ChassisSpeeds(
                translationalOutputSpeeds.getX(),
                translationalOutputSpeeds.getY(),
                rotationalFeedforward + rotationalFeedback);
        ChassisSpeeds robotRelativeOutput =
                ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeOutput, currentPose.getRotation());
        return HolonomicDriveSubsystem.constrainSpeeds(robotRelativeOutput);
    }

    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        xController.reset();
        yCOntroller.reset();
        rotationController.reset();
    }

    @Override
    public boolean isHolonomic() {
        return true;
    }
}
