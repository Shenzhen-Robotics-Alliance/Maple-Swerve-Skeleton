package frc.robot.utils.CompetitionFieldUtils.Simulations;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drive.OpponentRobotFollowPath;
import frc.robot.constants.DriveTrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CompetitionFieldUtils.Objects.Crescendo2024FieldObjects;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MaplePathPlannerLoader;

/**
 * simulates an opponent robot on field
 * in physics, the opponent robot behaves just the same as our own robot, it also follows the Holonomic Chassis Physics
 * the difference is, opponent robots are not controlled by the main gamepad
 * it is either controlled by another gamepad to simulate a defense robot
 * or can follow pre-generated paths to simulate opponent robots who are doing cycles
 * */
public class OpponentRobotSimulation extends HolonomicChassisSimulation implements HolonomicDriveSubsystem {
    /* if an opponent robot is not requested to be on field, it queens outside the field for performance */
    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
            new Pose2d(-6, 0, new Rotation2d()),
            new Pose2d(-5, 0, new Rotation2d()),
            new Pose2d(-4, 0, new Rotation2d()),
            new Pose2d(-3, 0, new Rotation2d()),
            new Pose2d(-2, 0, new Rotation2d())
    };
    public static final Pose2d[] ROBOTS_STARTING_POSITIONS = new Pose2d[] {
            new Pose2d(15, 6, Rotation2d.fromDegrees(180)),
            new Pose2d(15, 4, Rotation2d.fromDegrees(180)),
            new Pose2d(15, 2, Rotation2d.fromDegrees(180)),
            new Pose2d(1.6, 6, new Rotation2d()),
            new Pose2d(1.6, 4, new Rotation2d())
    };
    public static final RobotSimulationProfile opponentRobotProfile = new RobotSimulationProfile(
            4,
            12,
            Math.toRadians(360),
            Units.lbsToKilograms(125),
            DriveTrainConstants.BUMPER_WIDTH_METERS,
            DriveTrainConstants.BUMPER_LENGTH_METERS,
            1, 1
    );

    private final int robotID;
    private final SendableChooser<Command> behaviorChooser = new SendableChooser<>();
    private final Runnable disable;
    /**
     * @param id the id of the robot, 0 to 2, this determines where the robot "respawns"
     * */
    public OpponentRobotSimulation(int id, CompetitionFieldSimulation simulation) {
        super(opponentRobotProfile, ROBOT_QUEENING_POSITIONS[id]);
        this.robotID = id;
        this.disable = () -> {
            stop();
            setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[robotID]);
        };

        behaviorChooser.addOption("Disabled", Commands.runOnce(disable, this));
        behaviorChooser.setDefaultOption("Auto Cycle", this.getAutoCyleRepeadtelyCommand(
                switch (robotID) {
                    case 0 -> Commands.none();
                    case 3 -> Commands.runOnce(() -> simulation.addGamePiece(new Crescendo2024FieldObjects.FeedShotLowNote(
                            getPose().getTranslation(),
                            getFacing()
                    )));
                    case 4 -> Commands.runOnce(() -> simulation.getVisualizer().addGamePieceOnFly(new Crescendo2024FieldObjects.FeedShotHighNote(
                            getPose().getTranslation(),
                            FieldConstants.toCurrentAllianceTranslation(new Translation2d(2.27, 6.33)),
                            simulation
                    )));
                    default -> Commands.runOnce(() -> simulation.getVisualizer().addGamePieceOnFly(new Crescendo2024FieldObjects.NoteFlyingToSpeaker(
                            new Translation3d(getPose().getX(), getPose().getY(), 0.2),
                            0.2,
                            true
                    )));
                }
        ));
        final XboxController xboxController = new XboxController(1+robotID);
        behaviorChooser.addOption(
                "Joystick Control Left-Handed",
                getJoystickDrive(MapleJoystickDriveInput.leftHandedJoystick(xboxController))
        );
        behaviorChooser.addOption(
                "Joystick Control Right-Handed",
                getJoystickDrive(MapleJoystickDriveInput.rightHandedJoystick(xboxController))
        );
        behaviorChooser.onChange(Command::schedule);

        SmartDashboard.putData("FieldSimulation/OpponentRobot"+(robotID+1)+" Behavior", behaviorChooser);
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

    @Override public void updateSimulationSubTick(int iterationNum, double subPeriodSeconds) {
        super.simulateChassisBehaviorWithRobotRelativeSpeeds(speedSetPoint);
    }

    private static final PathConstraints constraints = new PathConstraints(3.5, 8, Math.toRadians(180), Math.toRadians(360));
    public Command getAutoCyleRepeadtelyCommand(Command toRunAtEndOfCycle) {
        final PathPlannerPath cycleForwardPath = MaplePathPlannerLoader.fromPathFile("other robot cycle path " + robotID, constraints),
                cycleBackwardPath = MaplePathPlannerLoader.fromPathFileReversed(
                        "other robot cycle path " + robotID,
                        constraints,
                        new GoalEndState(0, cycleForwardPath.getPreviewStartingHolonomicPose().getRotation()));
        final Command teleportToStartingPose = Commands.runOnce(() -> setSimulationWorldPose(cycleBackwardPath.getPreviewStartingHolonomicPose()), this),
                cycleForward = new OpponentRobotFollowPath(cycleForwardPath, FieldConstants::isSidePresentedAsRed, this),
                cycleBackward = new OpponentRobotFollowPath(cycleBackwardPath, FieldConstants::isSidePresentedAsRed, this);

        final Runnable end = () -> {
            stop();
            setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[robotID]);
        };

        final Command cycleRepeatedlyAndStop = new SequentialCommandGroup(
                teleportToStartingPose,
                new SequentialCommandGroup(
                        cycleBackward.withTimeout(8),
                        Commands.waitSeconds(1),
                        cycleForward
                                .andThen(Commands.waitSeconds(0.5).andThen(toRunAtEndOfCycle))
                                .withTimeout(8),
                        Commands.waitSeconds(0.5)
                ).repeatedly()
        ).finallyDo(end);
        cycleRepeatedlyAndStop.addRequirements(this);
        return cycleRepeatedlyAndStop;
    }

    public Command getJoystickDrive(MapleJoystickDriveInput joystickDriveInput) {
        final Pose2d startingPose = FieldConstants.toCurrentAlliancePose(ROBOTS_STARTING_POSITIONS[robotID]),
                queeningPose = ROBOT_QUEENING_POSITIONS[robotID];
        final Command teleportToStartingPose = Commands.runOnce(() -> setSimulationWorldPose(startingPose), this);
        Runnable end = () -> {
            setSimulationWorldPose(queeningPose);
            stop();
        };

        return new SequentialCommandGroup(
                teleportToStartingPose,
                Commands.run(() -> joystickDrivePeriod(joystickDriveInput), this)
        ).finallyDo(end);
    }

    private void joystickDrivePeriod(MapleJoystickDriveInput driveInput) {
        final ChassisSpeeds gamePadSpeeds = driveInput.getJoystickChassisSpeeds(4, 8),
                gamePadSpeedsInOurDriverStationReference = ChassisSpeeds.fromFieldRelativeSpeeds(gamePadSpeeds, new Rotation2d(Math.PI));
        HolonomicDriveSubsystem.super.runDriverStationCentricChassisSpeeds(gamePadSpeedsInOurDriverStationReference);
    }

    public void teleOpInit() {
        behaviorChooser.getSelected().schedule();
    }

    @Override
    public String getTypeName() {
        final boolean isOpponent = robotID < 3;
        return (isOpponent ? "Opponent" : "AlliancePartner") + "Robot";
    }
}
