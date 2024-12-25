// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main,
// Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import static edu.wpi.first.units.Units.*;

import frc.robot.constants.DriveTrainConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized to a random value.
 * The flywheel sims are not physically accurate, but provide a decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor, steerMotor;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        this.moduleSimulation = moduleSimulation;
        this.driveMotor = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(DriveTrainConstants.DRIVE_CURRENT_LIMIT_ANTI_SLIP);
        this.steerMotor = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(DriveTrainConstants.STEER_CURRENT_LIMIT);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveWheelFinalRevolutions =
                moduleSimulation.getDriveWheelFinalPosition().in(Revolutions);
        inputs.driveWheelFinalVelocityRevolutionsPerSec =
                moduleSimulation.getDriveWheelFinalSpeed().in(RevolutionsPerSecond);
        inputs.driveMotorAppliedVolts =
                moduleSimulation.getDriveMotorAppliedVoltage().in(Volts);
        inputs.driveMotorCurrentAmps =
                moduleSimulation.getDriveMotorStatorCurrent().in(Amps);

        inputs.steerFacing = moduleSimulation.getSteerAbsoluteFacing();
        inputs.steerVelocityRadPerSec =
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.steerMotorAppliedVolts =
                moduleSimulation.getSteerMotorAppliedVoltage().in(Volts);
        inputs.steerMotorCurrentAmps =
                moduleSimulation.getSteerMotorStatorCurrent().in(Amps);

        inputs.odometryDriveWheelRevolutions = Arrays.stream(moduleSimulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Revolutions))
                .toArray();
        inputs.odometrySteerPositions = moduleSimulation.getCachedSteerAbsolutePositions();

        inputs.hardwareConnected = true;
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.requestVoltage(Volts.of(volts));
    }

    @Override
    public void setSteerPowerPercent(double powerPercent) {
        steerMotor.requestVoltage(Volts.of(12).times(powerPercent));
    }
}
