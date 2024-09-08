// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.Arrays;

import static frc.robot.constants.DriveTrainConstants.*;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    public final SwerveModulePhysicsSimulationResults physicsSimulationResults;
    private final DCMotorSim steerSim;
    private double driveAppliedVolts = 0.0, steerAppliedVolts = 0.0;

    public ModuleIOSim() {
        this.steerSim = new DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA);

        this.physicsSimulationResults = new SwerveModulePhysicsSimulationResults();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = physicsSimulationResults.driveWheelFinalRevolutions;
        inputs.driveWheelFinalVelocityRevolutionsPerSec = Units.radiansToRotations(physicsSimulationResults.driveWheelFinalVelocityRadPerSec);
        inputs.driveMotorAppliedVolts = driveAppliedVolts;
        inputs.driveMotorCurrentAmps = Math.abs(DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec,
                driveAppliedVolts
        ));

        inputs.steerFacing = Rotation2d.fromRadians(steerSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerMotorAppliedVolts = steerAppliedVolts;
        inputs.steerMotorCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.odometryDriveWheelRevolutions = Arrays.copyOf(
                physicsSimulationResults.odometryDriveWheelRevolutions,
                SIMULATION_TICKS_IN_1_PERIOD
        );
        inputs.odometrySteerPositions = Arrays.copyOf(
                physicsSimulationResults.odometrySteerPositions,
                SIMULATION_TICKS_IN_1_PERIOD
        );

        inputs.hardwareConnected = true;
    }


    @Override
    public void setDriveVoltage(double volts) {
        driveAppliedVolts = volts;
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        steerAppliedVolts = (powerPercent * 12);
        steerSim.setInputVoltage(Math.abs(steerAppliedVolts) > STEER_FRICTION_VOLTAGE ? steerAppliedVolts : 0);
    }

    public void updateSim(double periodSecs) {
        steerSim.update(periodSecs);
    }

    public double getSimulationTorque() {
        return DRIVE_MOTOR.getTorque(DRIVE_MOTOR.getCurrent(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec * DRIVE_GEAR_RATIO,
                driveAppliedVolts
        ));
    }

    public double getAppliedVolts() {
        return driveAppliedVolts;
    }

    public Rotation2d getSimulationSteerFacing() {
        return Rotation2d.fromRadians(steerSim.getAngularPositionRad());
    }

    public SwerveModuleState getSimulationSwerveState() {
        return new SwerveModuleState(
                physicsSimulationResults.driveWheelFinalVelocityRadPerSec * WHEEL_RADIUS_METERS,
                getSimulationSteerFacing()
        );
    }

    public SwerveModuleState getDesiredSwerveState() {
        return new SwerveModuleState(
                driveAppliedVolts * CHASSIS_MAX_VELOCITY,
                getSimulationSteerFacing()
        );
    }

    /**
     * this replaces DC Motor Sim for drive wheels
     * */
    public static class SwerveModulePhysicsSimulationResults {
        public double
                driveWheelFinalRevolutions = 0,
                driveWheelFinalVelocityRadPerSec = 0;

        public final double[] odometryDriveWheelRevolutions =
                new double[SIMULATION_TICKS_IN_1_PERIOD];
        public final Rotation2d[] odometrySteerPositions =
                new Rotation2d[SIMULATION_TICKS_IN_1_PERIOD];

        public SwerveModulePhysicsSimulationResults() {
            Arrays.fill(odometrySteerPositions, new Rotation2d());
            Arrays.fill(odometryDriveWheelRevolutions, 0);
        }
    }
}
