package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.DriveControlLoops;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.HolonomicDriveSubsystem;
import frc.robot.utils.CustomPIDs.MaplePIDController;
import frc.robot.utils.MapleJoystickDriveInput;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.subsystems.drive.HolonomicDriveSubsystem.isZero;
import static frc.robot.constants.JoystickConfigs.*;

public class JoystickDrive extends Command {
    protected MapleJoystickDriveInput input;
    private final BooleanSupplier useDriverStationCentricSwitch;
    private final Supplier<Integer> povButtonSupplier;
    private final HolonomicDriveSubsystem driveSubsystem;

    private final TrapezoidProfile rotationProfile;
    private final PIDController chassisRotationController;
    private TrapezoidProfile.State currentRotationState;


    protected final Timer previousChassisUsageTimer, previousRotationalInputTimer;
    private ChassisSpeeds currentPilotInputSpeeds;
    protected Rotation2d currentRotationMaintenanceSetpoint;

    private double translationalSensitivity, rotationalSensitivity;
    public JoystickDrive(MapleJoystickDriveInput input, BooleanSupplier useDriverStationCentricSwitch, Supplier<Integer> povButtonSupplier, HolonomicDriveSubsystem driveSubsystem) {
        super();
        this.input = input;
        this.useDriverStationCentricSwitch = useDriverStationCentricSwitch;
        this.povButtonSupplier = povButtonSupplier;
        this.driveSubsystem = driveSubsystem;
        this.previousChassisUsageTimer = new Timer();
        this.previousChassisUsageTimer.start();
        this.previousRotationalInputTimer = new Timer();
        this.previousRotationalInputTimer.start();

        this.rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                driveSubsystem.getChassisMaxAngularVelocity() * 0.7,
                driveSubsystem.getChassisMaxAngularAccelerationRadPerSecSq()
        ));
        this.chassisRotationController = new MaplePIDController(DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP);
        this.chassisRotationController.enableContinuousInput(-Math.PI, Math.PI);

        super.addRequirements(driveSubsystem);
        resetSensitivity();
    }

    @Override
    public void initialize() {
        this.previousChassisUsageTimer.reset();
        this.previousRotationalInputTimer.reset();
        this.currentPilotInputSpeeds = new ChassisSpeeds();
        this.currentRotationMaintenanceSetpoint = driveSubsystem.getRawGyroYaw();

        syncRotationControllerToCurrentRobotFacing();
    }

    public void syncRotationControllerToCurrentRobotFacing() {
        this.chassisRotationController.reset();
        this.currentRotationState = new TrapezoidProfile.State(
                driveSubsystem.getRawGyroYaw().getRadians(),
                driveSubsystem.getMeasuredChassisSpeedsRobotRelative().omegaRadiansPerSecond
        );
    }

    @Override
    public void execute() {
        if (input == null) return;

        final ChassisSpeeds newestPilotInputSpeed = input.getJoystickChassisSpeeds(
                driveSubsystem.getChassisMaxLinearVelocityMetersPerSec() * translationalSensitivity,
                driveSubsystem.getChassisMaxAngularVelocity() * rotationalSensitivity
        );
        currentPilotInputSpeeds = driveSubsystem.constrainAcceleration(
                currentPilotInputSpeeds,
                newestPilotInputSpeed,
                Robot.defaultPeriodSecs
        );

        if (Math.abs(currentPilotInputSpeeds.omegaRadiansPerSecond) > 0.05) {
            previousRotationalInputTimer.reset();
            syncRotationControllerToCurrentRobotFacing();
        }

        if (povButtonSupplier.get() != -1)
            setCurrentRotationalMaintenance(
                    FieldConstants
                            .getDriverStationFacing()
                            .minus(Rotation2d.fromDegrees(povButtonSupplier.get()))
            );

        final ChassisSpeeds chassisSpeedsWithRotationMaintenance;
        this.currentRotationState = rotationProfile.calculate(
                Robot.defaultPeriodSecs,
                currentRotationState,
                getRotationProfileGoal()
        );
        final double rotationCorrectionAngularVelocity = chassisRotationController.calculate(
                driveSubsystem.getRawGyroYaw().getRadians(),
                currentRotationState.position
        ) + currentRotationState.velocity;
        Logger.recordOutput("previousRotationalInputTimer.get()", previousRotationalInputTimer.get());
        if (previousRotationalInputTimer.get() > TIME_ACTIVATE_ROTATION_MAINTENANCE_AFTER_NO_ROTATIONAL_INPUT_SECONDS)
            chassisSpeedsWithRotationMaintenance = new ChassisSpeeds(
                    currentPilotInputSpeeds.vxMetersPerSecond, currentPilotInputSpeeds.vyMetersPerSecond,
                    rotationCorrectionAngularVelocity
            );
        else {
            chassisSpeedsWithRotationMaintenance = currentPilotInputSpeeds;
            Logger.recordOutput("pilot input speeds", currentPilotInputSpeeds);
            currentRotationMaintenanceSetpoint = driveSubsystem.getRawGyroYaw();
        }

        Logger.recordOutput("JoystickDrive/rotation maintenance set-point (deg)", currentRotationMaintenanceSetpoint.getDegrees());
        Logger.recordOutput("JoystickDrive/previous rotational input time", previousRotationalInputTimer.get());
        Logger.recordOutput("JoystickDrive/rotation closed loop velocity (deg per sec)", Math.toDegrees(rotationCorrectionAngularVelocity));

        if (!isZero(chassisSpeedsWithRotationMaintenance))
            previousChassisUsageTimer.reset();
        Logger.recordOutput("JoystickDrive/current pilot input speeds", currentPilotInputSpeeds.toString());

        if (previousChassisUsageTimer.hasElapsed(NON_USAGE_TIME_RESET_WHEELS)) {
            driveSubsystem.stop();
            return;
        }

        if (useDriverStationCentricSwitch.getAsBoolean())
            driveSubsystem.runDriverStationCentricChassisSpeeds(chassisSpeedsWithRotationMaintenance, true);
        else
            driveSubsystem.runRobotCentricChassisSpeeds(chassisSpeedsWithRotationMaintenance, true);

        Logger.recordOutput("rotationMaintain", new Pose2d(
                driveSubsystem.getPose().getTranslation(),
                Rotation2d.fromRadians(currentRotationState.position)
        ));
    }

    private TrapezoidProfile.State getRotationProfileGoal() {
        final Rotation2d currentRotationStateRotation = Rotation2d.fromRadians(currentRotationState.position),
                difference = currentRotationMaintenanceSetpoint.minus(currentRotationStateRotation);
        final double goalRotationRad = currentRotationState.position + difference.getRadians();
        return new TrapezoidProfile.State(goalRotationRad, 0);
    }

    public void setCurrentRotationalMaintenance(Rotation2d setPointAbsoluteFacing) {
        final Rotation2d gyroReadingBiasFromActualFacing = driveSubsystem.getRawGyroYaw().minus(driveSubsystem.getFacing());
        this.currentRotationMaintenanceSetpoint = setPointAbsoluteFacing.rotateBy(gyroReadingBiasFromActualFacing);
    }

    public void setSensitivity(double translationalSensitivity, double rotationalSensitivity) {
        this.translationalSensitivity = translationalSensitivity;
        this.rotationalSensitivity = rotationalSensitivity;
    }

    public void resetSensitivity() {
        setSensitivity(DEFAULT_TRANSLATIONAL_SENSITIVITY, DEFAULT_ROTATIONAL_SENSITIVITY);
    }
}
