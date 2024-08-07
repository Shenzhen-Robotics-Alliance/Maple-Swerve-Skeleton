// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.Config.MapleConfigFile;

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
    private final double GEAR_RATIO;
    public final SwerveModulePhysicsSimulationResults physicsSimulationResults;
    private final DCMotorSim driveSim, steerSim;
    private double driveAppliedVolts = 0.0, steerAppliedVolts = 0.0;

    public ModuleIOSim(MapleConfigFile.ConfigBlock generalConfigs) {
        this.GEAR_RATIO = generalConfigs.getDoubleConfig("overallGearRatio");
        this.driveSim = new DCMotorSim(DRIVE_MOTOR, GEAR_RATIO, DRIVE_WHEEL_ROTTER_INERTIA);
        this.steerSim = new DCMotorSim(STEER_MOTOR, STEER_GEAR_RATIO, STEER_INERTIA);
        this.physicsSimulationResults = new SwerveModulePhysicsSimulationResults();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions = physicsSimulationResults.driveWheelFinalRevolutions;
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

        inputs.hardwareConnected = true;
    }


    @Override
    public void setDriveSpeedPercent(double speedPercent) {
        driveSim.setInputVoltage(
                driveAppliedVolts = (speedPercent * 12)
        );
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        steerSim.setInputVoltage(
            steerAppliedVolts = (powerPercent * 12)
        );
    }

    public void updateSim(double periodSecs) {
        steerSim.update(periodSecs);
        driveSim.update(periodSecs);
    }

    /**
     * gets the swerve state, assuming that the chassis is allowed to move freely on field (not hitting anything)
     * @return the swerve state, in percent full speed
     * */
    public SwerveModuleState getFreeSwerveSpeed(double robotMaximumFloorSpeed) {
        return new SwerveModuleState(
                driveSim.getAngularVelocityRPM() * GEAR_RATIO
                        / DRIVE_MOTOR_FREE_FINAL_SPEED_RPM
                        * robotMaximumFloorSpeed,
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
