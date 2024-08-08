package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.MapleJoystickDriveInput;
import frc.robot.utils.MapleShooterOptimization;
import frc.robot.utils.MechanismControl.MaplePIDController;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class JoystickDriveAndAimAtTarget extends JoystickDrive {
    private final Supplier<Translation2d> targetPositionSupplier;
    private final DoubleSupplier inAdvanceTimeSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;
    private final PIDController chassisRotationController;
    public JoystickDriveAndAimAtTarget(MapleJoystickDriveInput input, HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier, MapleShooterOptimization shooterOptimization) {
        this(input, driveSubsystem, targetPositionSupplier,
                () -> shooterOptimization.getFlightTimeSeconds(targetPositionSupplier.get(), driveSubsystem.getPose().getTranslation()));
    }

    public JoystickDriveAndAimAtTarget(MapleJoystickDriveInput input, HolonomicDriveSubsystem driveSubsystem, Supplier<Translation2d> targetPositionSupplier, DoubleSupplier inAdvanceTimeSupplier) {
        super(
                null,
                () -> true,
                driveSubsystem);
        this.targetPositionSupplier = targetPositionSupplier;
        this.inAdvanceTimeSupplier = inAdvanceTimeSupplier;
        this.chassisRotationController = new MaplePIDController(
                Constants.SwerveDriveChassisConfigs.chassisRotationalPIDConfig
        );

        this.driveSubsystem = driveSubsystem;
        super.input = new MapleJoystickDriveInput(
                input.joystickXSupplier, input.joystickYSupplier,
                this::getRotationalCorrectionPowerPercentClockWise
        );
    }

    @Override
    public void initialize() {
        this.chassisRotationController.calculate(driveSubsystem.getRawGyroYaw().getRadians());
        super.initialize();
    }

    @Override
    public void execute() {
        /* disable auto stop and original rotation maintenance */
        previousChassisUsageTimer.reset();
        previousRotationalInputTimer.reset();

        super.execute();
    }

    public static double FEED_FORWARD_RATE = 1.3;
    public double getRotationalCorrectionPowerPercentClockWise() {
        final Translation2d robotPosition = driveSubsystem.getPose().getTranslation();
        final ChassisSpeeds robotVelocityFieldRelative = driveSubsystem.getMeasuredChassisSpeedsFieldRelative();
        final Translation2d robotPositionAfterDt = robotPosition.plus(new Translation2d(
                robotVelocityFieldRelative.vxMetersPerSecond * Robot.defaultPeriodSecs,
                robotVelocityFieldRelative.vyMetersPerSecond * Robot.defaultPeriodSecs
        ));
        final Translation2d targetPosition = targetPositionSupplier.get(),
                robotNewPosition = robotPosition.plus(new Translation2d(
                        robotVelocityFieldRelative.vxMetersPerSecond * inAdvanceTimeSupplier.getAsDouble(),
                        robotVelocityFieldRelative.vyMetersPerSecond * inAdvanceTimeSupplier.getAsDouble()
                ).times(inAdvanceTimeSupplier.getAsDouble()));
        final Rotation2d targetedFacingInAdvance = targetPosition.minus(robotNewPosition).getAngle(),
                targetFacingNow = targetPosition.minus(robotPosition).getAngle(),
                targetFacingAfterDT = targetPosition.minus(robotPositionAfterDt).getAngle();
        final double targetedFacingChangeRateRadPerSec =
                targetFacingAfterDT.minus(targetFacingNow).getRadians()
                        / Robot.defaultPeriodSecs;
        Logger.recordOutput("Drive/Face To Target Rotation (Deg)",targetedFacingInAdvance.getDegrees());

        final double feedBackRotationalSpeed = chassisRotationController.calculate(
                driveSubsystem.getFacing().getRadians(),
                targetedFacingInAdvance.getRadians()),
                feedForwardRotationalSpeed = targetedFacingChangeRateRadPerSec
                        * FEED_FORWARD_RATE;
        return  (feedForwardRotationalSpeed + feedBackRotationalSpeed) / -driveSubsystem.getChassisMaxAngularVelocity();
    }
}
