package frc.robot.utils.CompetitionFieldUtils.Simulation;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.FollowPath;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import org.ejml.simple.UnsupportedOperation;

/**
 * simulates an opponent robot on field
 * in physics, the opponent robot behaves just the same as our own robot, it also follows the Holonomic Chassis Physics
 * the difference is, opponent robots are not controlled by the main gamepad
 * it is either controlled by another gamepad to simulate a defense robot
 * or can follow pre-generated paths to simulate opponent robots who are doing cycles
 * */
public class OpponentRobotSimulation extends HolonomicChassisSimulation implements HolonomicDriveSubsystem {
    public enum Behavior {
        JOYSTICK_CONTROL,
        AUTO_CYCLE,
        QUEEN
    }
    /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
    public static final Pose2d robotQueeningPosition = new Pose2d(-5, 5, new Rotation2d());
    public static final HolonomicChassisSimulation.RobotProfile opponentRobotProfile = new RobotProfile(
            4,
            12,
            Math.toRadians(360),
            Units.lbsToKilograms(125),
            Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_WIDTH_METERS,
            Constants.RobotPhysicsSimulationConfigs.DEFAULT_BUMPER_LENGTH_METERS
    );

    private final int id;

    /**
     * @param id the id of the robot, 0 to 2, this determines where the robot "respawns"
     * */
    public OpponentRobotSimulation(int id) {
        super(opponentRobotProfile, robotQueeningPosition);
        if (id >= 3)
            throw new IllegalArgumentException("id must be 0~2");
        this.id = id;
    }

    private ChassisSpeeds speedSetPoint = new ChassisSpeeds();
    @Override
    public void runRawChassisSpeeds(ChassisSpeeds speeds) {
        this.speedSetPoint = speeds;
    }

    @Override
    public Pose2d getPose() {
        return super.getObjectOnFieldPose2d();
    }

    @Override
    public void setPose(Pose2d currentPose) {
        super.setSimulationWorldPose(currentPose);
    }

    @Override public double getChassisMaxLinearVelocityMetersPerSec() {return profile.robotMaxVelocity;}
    @Override public double getChassisMaxAccelerationMetersPerSecSq() {return profile.robotMaxAcceleration;}
    @Override public double getChassisMaxAngularVelocity() {return profile.maxAngularVelocity;}
    @Override public double getChassisMaxAngularAccelerationRadPerSecSq() {return profile.maxAngularAcceleration;}

    @Override public void addVisionMeasurement(Pose2d visionPose, double timestamp) {throw new UnsupportedOperation("an opponent robot does not support vision measurement"); }
    @Override public void updateSimulationSubPeriod(int iterationNum, double subPeriodSeconds) {
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(speedSetPoint);
    }

    public Command getAutoCyleCommand() {
        final PathPlannerPath cyclePathRaw = PathPlannerPath.fromPathFile("opponent cycle path " + id),
                cyclePath = Constants.isSidePresentedAsRed() ? cyclePathRaw.flipPath() : cyclePathRaw,
                cyclePathReversed = FollowPath.reversePath(
                    cyclePath,
                    new GoalEndState(0, cyclePath.getPreviewStartingHolonomicPose().getRotation())
                );
        cyclePath.preventFlipping = cyclePathReversed.preventFlipping = true;
        final Command teleportToStartingPose = Commands.runOnce(
                () -> setSimulationWorldPose(cyclePathReversed.getPreviewStartingHolonomicPose())),
                cycleForward = new FollowPath(cyclePath, () -> false, this),
                cycleBackWards = new FollowPath(cyclePathReversed, () -> false, this);
        return new SequentialCommandGroup(
                teleportToStartingPose,
                new SequentialCommandGroup(
                        cycleBackWards,
                        cycleForward
                ).repeatedly()
        ).finallyDo(this::stop);
    }

    /**
     * a method to test the driving physics
     * just for testing
     * in the formal code, we should be using holonomic drive commands
     * */
    @Deprecated
    public void testDrivingPhysicsWithJoystick(XboxController xboxController) {
        final MapleJoystickDriveInput mapleJoystickDriveInput = MapleJoystickDriveInput.leftHandedJoystick(xboxController);
        final ChassisSpeeds gamePadSpeeds = mapleJoystickDriveInput.getJoystickChassisSpeeds(5, 10);
        HolonomicDriveSubsystem.super.runDriverStationCentricChassisSpeeds(gamePadSpeeds);
    }
}
