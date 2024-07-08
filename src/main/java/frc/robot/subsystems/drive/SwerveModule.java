// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.MechanismControl.MapleSimplePIDController;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends MapleSubsystem {
    private final ModuleIO io;
    private final String name;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final SimpleMotorFeedforward driveOpenLoop;
    private final MapleSimplePIDController turnCloseLoop;
    private Rotation2d angleSetpoint;
    private double speedSetpoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    public SwerveModule(ModuleIO io, String name) {
        super("Module-" + name);
        this.io = io;
        this.name = name;

        driveOpenLoop = new SimpleMotorFeedforward(0.1, 0.13);
        turnCloseLoop = new MapleSimplePIDController(Constants.SwerveModuleConfigs.steerHeadingCloseLoopConfig, 0);

        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    @Override
    public void onReset() {
        angleSetpoint = new Rotation2d();
        speedSetpoint = 0;
    }

    public void updateOdometryInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module-" + name, inputs);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        updateOdometryPositions();

        if (enabled) {
            runDriveOpenLoop();
            runSteerCloseLoop();
        }
    }

    private void updateOdometryPositions() {
        odometryPositions = new SwerveModulePosition[inputs.odometryDriveWheelRevolutions.length];
        for (int i = 0; i < odometryPositions.length; i++) {
            double positionMeters = toDrivePositionMeters(inputs.odometryDriveWheelRevolutions[i]);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    private void runSteerCloseLoop() {
        turnCloseLoop.setDesiredPosition(angleSetpoint.getRadians());
        io.setSteerPower(turnCloseLoop.getMotorPower(
                getSteerVelocityRadPerSec(),
                getSteerFacing().getRadians()
        ));
    }

    private void runDriveOpenLoop() {
        final double
                CURRENT_STEER_FACING_TO_DESIRED_FACING_DIFFERENCE = angleSetpoint.minus(getSteerFacing()).getRadians(),
                DESIRED_VELOCITY_PROJECTION_RATIO_TO_CURRENT_STEER_FACING = Math.cos(
                        CURRENT_STEER_FACING_TO_DESIRED_FACING_DIFFERENCE
                ),
                adjustSpeedSetpoint = speedSetpoint * DESIRED_VELOCITY_PROJECTION_RATIO_TO_CURRENT_STEER_FACING,
                velocitySetPointRadPerSec = adjustSpeedSetpoint / Constants.SwerveModuleConfigs.WHEEL_RADIUS;
        io.setDrivePower(driveOpenLoop.calculate(velocitySetPointRadPerSec));
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public SwerveModuleState requestSetPoint(SwerveModuleState state) {
        var optimizedState = SwerveModuleState.optimize(state, getSteerFacing());

        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    public SwerveModuleState requestXFormationSetpoint() {
        return requestSetPoint(new SwerveModuleState()); // TODO write this method
    }

    @Override
    public void onDisable() {
        io.setSteerPower(0);
        io.setDrivePower(0);
        io.setDriveBrakeMode(false);
        io.setSteerBrakeMode(false);
    }

    @Override
    public void onEnable() {
        io.setDriveBrakeMode(true);
        io.setSteerBrakeMode(true);
    }

    /**
     * Returns the current turn angle of the module.
     */
    public Rotation2d getSteerFacing() {
        return inputs.steerFacing;
    }

    public double getSteerVelocityRadPerSec() {
        return inputs.steerVelocityRadPerSec;
    }

    /**
     * Returns the current drive position of the module in meters.
     */
    public double getDrivePositionMeters() {
        return toDrivePositionMeters(inputs.driveWheelFinalRevolutions);
    }

    private double toDrivePositionMeters(double driveWheelRevolutions) {
        return Units.rotationsToRadians(driveWheelRevolutions) * Constants.SwerveModuleConfigs.WHEEL_RADIUS;
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getDriveVelocityMetersPerSec() {
        return toDrivePositionMeters(inputs.driveWheelFinalVelocityRevolutionsPerSec);
    }

    /**
     * Returns the module position (turn angle and drive position).
     */
    public SwerveModulePosition getLatestPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerFacing());
    }

    /**
     * Returns the module state (turn angle and drive velocity).
     */
    public SwerveModuleState getMeasuredState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSec(), getSteerFacing());
    }

    /**
     * Returns the module positions received this cycle.
     */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }
}
