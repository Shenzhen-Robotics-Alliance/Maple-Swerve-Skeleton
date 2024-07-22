// Original Source:
// https://github.com/Mechanical-Advantage/AdvantageKit/tree/main/example_projects/advanced_swerve_drive/src/main, Copyright 2021-2024 FRC 6328
// Modified by 5516 Iron Maple https://github.com/Shenzhen-Robotics-Alliance/

package frc.robot.subsystems.drive.IO;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim steerSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private double driveAppliedVolts = 0.0;
    private double steerAppliedVolts = 0.0;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        driveSim.update(Robot.defaultPeriodSecs);
        steerSim.update(Robot.defaultPeriodSecs);

        inputs.driveWheelFinalRevolutions = Units.radiansToRotations(driveSim.getAngularPositionRad());
        inputs.driveWheelFinalVelocityRevolutionsPerSec = driveSim.getAngularVelocityRPM() / 60.0;
        inputs.driveMotorAppliedVolts = driveAppliedVolts;
        inputs.driveMotorCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        inputs.steerFacing = Rotation2d.fromRadians(steerSim.getAngularPositionRad());
        inputs.steerVelocityRadPerSec = steerSim.getAngularVelocityRadPerSec();
        inputs.steerMotorAppliedVolts = steerAppliedVolts;
        inputs.steerMotorCurrentAmps = Math.abs(steerSim.getCurrentDrawAmps());

        inputs.odometryDriveWheelRevolutions = new double[]{inputs.driveWheelFinalRevolutions};
        inputs.odometrySteerPositions = new Rotation2d[]{inputs.steerFacing};
    }

    @Override
    public void setDrivePower(double power) {
        driveSim.setInputVoltage(power * 12);
    }

    @Override
    public void setSteerPower(double power) {
        steerSim.setInputVoltage(power * 12);
    }
}
