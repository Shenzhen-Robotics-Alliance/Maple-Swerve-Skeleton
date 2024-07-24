// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.Arrays;

import static frc.robot.Constants.RobotPhysicsSimulationConfigs.*;
import static frc.robot.Constants.ChassisDefaultConfigs.*;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    public final SwerveModulePhysicsSimulationResults physicsSimulationResults = new SwerveModulePhysicsSimulationResults();
    private final DCMotorSim driveSim = new DCMotorSim(DRIVE_MOTOR, DEFAULT_GEAR_RATIO, 0.025);
    private final DCMotorSim steerSim = new DCMotorSim(STEER_MOTOR, 150.0 / 7.0, 0.004);
    private double driveAppliedVolts = 0.0, steerAppliedVolts = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = Units.radiansToRotations(physicsSimulationResults.driveWheelFinalRevolutions);
        inputs.driveWheelFinalVelocityRevolutionsPerSec = physicsSimulationResults.driveWheelFinalVelocityRevolutionsPerSec;
        inputs.driveMotorAppliedVolts = driveAppliedVolts;
        inputs.driveMotorCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerFacing = Rotation2d.fromRadians(steerSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerMotorAppliedVolts = steerAppliedVolts;
        inputs.steerMotorCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.odometryDriveWheelRevolutions = Arrays.copyOf(
                physicsSimulationResults.odometryDriveWheelRevolutions,
                SIM_ITERATIONS_PER_ROBOT_PERIOD
        );
        inputs.odometrySteerPositions = Arrays.copyOf(
                physicsSimulationResults.odometrySteerPositions,
                SIM_ITERATIONS_PER_ROBOT_PERIOD
        );
    }

    @Override
    public void setDrivePower(double power) {
        driveSim.setInputVoltage(
                driveAppliedVolts = (power * 12)
        );
    }

    @Override
    public void setSteerPower(double power) {
        steerSim.setInputVoltage(
                steerAppliedVolts = (power * 12)
        );
    }

    public void updateSim(double periodSecs) {
        steerSim.update(periodSecs);
        driveSim.update(periodSecs);
    }

    /**
     * gets the swerve state, assuming that the chassis is allowed to move freely on field (not hitting anything)
     * @return the swerve state, in meters/second
     * */
    public SwerveModuleState getFreeSwerveSpeed() {
        return new SwerveModuleState(
                driveSim.getAngularVelocityRPM()
                        / DRIVE_MOTOR_FREE_FINAL_SPEED_RPM
                        * DEFAULT_MAX_VELOCITY_METERS_PER_SECOND,
                Rotation2d.fromRadians(steerSim.getAngularPositionRad())
        );
    }

    /**
     * this replaces DC Motor Sim for drive wheels
     * */
    public static class SwerveModulePhysicsSimulationResults {
        public double
                driveWheelFinalRevolutions = 0,
                driveWheelFinalVelocityRevolutionsPerSec = 0;

        public final double[] odometryDriveWheelRevolutions =
                new double[SIM_ITERATIONS_PER_ROBOT_PERIOD];
        public final Rotation2d[] odometrySteerPositions =
                new Rotation2d[SIM_ITERATIONS_PER_ROBOT_PERIOD];

        public SwerveModulePhysicsSimulationResults() {
            Arrays.fill(odometrySteerPositions, new Rotation2d());
            Arrays.fill(odometryDriveWheelRevolutions, 0);
        }
    }
}
