package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * <h1>Custom Drive Command</h1>
 * <p>The chassis will automatically face to a target on field (eg. the speaker) while the pilot controls its movements</p>
 * The chassis will also adjust its facing in-advance, with respect to the flight time calculated from {@link MapleShooterOptimization} (this is for shooting-on-the-move)
 * */
public class JoystickDriveAndAimAtTarget extends Command {
    private final MapleJoystickDriveInput input;
    private final Supplier<Translation2d> targetPositionSupplier;
    private final MapleShooterOptimization shooterOptimization;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final PIDController chassisRotationController;

    private final double pilotInputMultiplier;

    public JoystickDriveAndAimAtTarget(MapleJoystickDriveInput input, HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier, MapleShooterOptimization shooterOptimization, double pilotInputMultiplier) {
        this.targetPositionSupplier = targetPositionSupplier;
        this.shooterOptimization = shooterOptimization;
        this.pilotInputMultiplier = pilotInputMultiplier;
        this.chassisRotationController = new MaplePIDController(
                DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP
        );

        this.driveSubsystem = driveSubsystem;
        this.input = new MapleJoystickDriveInput(
                input.joystickXSupplier,
                input.joystickYSupplier,
                () -> 0
        );
    }

    @Override
    public void initialize() {
        this.chassisRotationController.calculate(driveSubsystem.getRawGyroYaw().getRadians());
        this.chassisRotationInPosition = false;
        SwerveDrive.acceptRotationalMeasurement = false;
    }

    @Override
    public void execute() {
        final ChassisSpeeds pilotInputSpeeds = input.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec(), driveSubsystem.getChassisMaxAngularVelocity())
                .times(pilotInputMultiplier),
                chassisSpeeds = pilotInputSpeeds.plus(new ChassisSpeeds(
                        0, 0,
                        getRotationalCorrectionVelocityRadPerSec()
                ));

        driveSubsystem.runDriverStationCentricChassisSpeeds(chassisSpeeds, false);
        super.execute();
    }

    public static double FEED_FORWARD_RATE = 1;
    public double getRotationalCorrectionVelocityRadPerSec() {
        final Translation2d robotPosition = driveSubsystem.getPose().getTranslation();
        final ChassisSpeeds robotVelocityFieldRelative = driveSubsystem.getMeasuredChassisSpeedsFieldRelative();
        final Translation2d robotPositionAfterDt = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * Robot.defaultPeriodSecs,
                robotVelocityFieldRelative.vyMetersPerSecond * Robot.defaultPeriodSecs
        ));

        final Rotation2d targetedFacing =
                shooterOptimization.getShooterFacing(
                        targetPositionSupplier.get(),
                        robotPosition,
                        robotVelocityFieldRelative
                ),
                /* to calculate the derivative of target facing */
                targetedFacingAfterDT = shooterOptimization.getShooterFacing(
                        targetPositionSupplier.get(),
                        robotPositionAfterDt,
                        robotVelocityFieldRelative
                );
        final double targetedFacingChangeRateRadPerSec =
                targetedFacingAfterDT.minus(targetedFacing).getRadians()
                        / Robot.defaultPeriodSecs;

        final double feedBackRotationalSpeed = chassisRotationController.calculate(
                driveSubsystem.getFacing().getRadians(),
                targetedFacing.getRadians()
        ),
                feedForwardRotationalSpeed = MathUtil.applyDeadband(
                        targetedFacingChangeRateRadPerSec * FEED_FORWARD_RATE,
                        Math.toRadians(30),
                        Double.POSITIVE_INFINITY
                );

        final Rotation2d chassisRotationalError = targetedFacing
                .minus(driveSubsystem.getFacing());
        this.chassisRotationInPosition = Math.abs(chassisRotationalError.getRadians())
                < DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP.errorTolerance;

        SmartDashboard.putBoolean("Chassis Rotation Aiming Target Reached", chassisRotationInPosition);
        SmartDashboard.putNumber("Chassis Rotation Aiming Error", chassisRotationalError.getDegrees());
        Logger.recordOutput("DriveAndAimAtTarget/Aim At Target Rational Error (Deg)", chassisRotationalError.getDegrees());
        Logger.recordOutput("DriveAndAimAtTarget/Rotation Target (Deg)", targetedFacing.getDegrees());
        Logger.recordOutput("DriveAndAimAtTarget/FeedForwardSpeed (Deg per Sec)", Math.toDegrees(feedForwardRotationalSpeed));
        Logger.recordOutput("DriveAndAimAtTarget/FeedBackSpeed (Deg per Sec)", Math.toDegrees(feedBackRotationalSpeed));

        return feedForwardRotationalSpeed + feedBackRotationalSpeed;
    }

    private boolean chassisRotationInPosition;
    public boolean chassisRotationInPosition() {
        return chassisRotationInPosition;
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDrive.acceptRotationalMeasurement = true;
    }
}
