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
import frc.robot.utils.MechanismControlHelpers.MapleSimplePIDController;
import org.littletonrobotics.junction.Logger;

public class Module extends MapleSubsystem {
    private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveOpenLoop;
    private final MapleSimplePIDController turnCloseLoop;
    private Rotation2d angleSetpoint;
    private double speedSetpoint;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[]{};

    public Module(ModuleIO io, int index) {
        super("Module" + index);
        this.io = io;
        this.index = index;

        driveOpenLoop = new SimpleMotorFeedforward(0.1, 0.13);
        turnCloseLoop = new MapleSimplePIDController(Constants.SwerveModuleConfigs.steerHeadingCloseLoopConfig, 0);

        CommandScheduler.getInstance().unregisterSubsystem(this);
    }

    @Override
    public void onReset() {
        angleSetpoint = new Rotation2d();
        speedSetpoint = 0;
    }

    public void fetchOdometryInputs() {
        long nanos = System.nanoTime();
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
        Logger.recordOutput(Constants.LogConfigs.SYSTEM_PERFORMANCE_PATH + "Module" + index + "/IO time", (System.nanoTime() - nanos) * 0.000001);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        long nanos = System.nanoTime();
        updateOdometryPositions();
        Logger.recordOutput(Constants.LogConfigs.SYSTEM_PERFORMANCE_PATH + "Module" + index + "/Odometry Time", (System.nanoTime() - nanos) * 0.000001);

        nanos = System.nanoTime();
        if (enabled) runDriveOpenLoop();
        if (enabled) runSteerCloseLoop();
        Logger.recordOutput(Constants.LogConfigs.SYSTEM_PERFORMANCE_PATH + "Module" + index + "/Close Loop Time", (System.nanoTime() - nanos) * 0.000001);
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
                velocitySetPointRadPerSec = adjustSpeedSetpoint / WHEEL_RADIUS;
        io.setDrivePower(driveOpenLoop.calculate(velocitySetPointRadPerSec));
    }

    /**
     * Runs the module with the specified setpoint state. Returns the optimized state.
     */
    public SwerveModuleState runSetpoint(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getSteerFacing());

        // Update setpoints, controllers run in "periodic"
        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    @Override
    public void onDisable() {
        io.setSteerPower(0);
        io.setDrivePower(0);
        io.setDriveBrakeMode(false);
        io.setTurnBrakeMode(false);
    }

    @Override
    public void onEnable() {
        io.setDriveBrakeMode(true);
        io.setTurnBrakeMode(true);
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
        return Units.rotationsToRadians(driveWheelRevolutions) * WHEEL_RADIUS;
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
