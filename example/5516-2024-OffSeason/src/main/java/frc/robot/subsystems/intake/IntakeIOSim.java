package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.constants.DriveTrainConstants;
import frc.robot.utils.CompetitionFieldUtils.Simulations.IntakeSimulation;

public class IntakeIOSim extends IntakeSimulation implements IntakeIO {
    /* running the intake in full voltage for 0.5 seconds is what it takes to get the note from bottom to top  */
    private static final double VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_IN_POSITION = 4 * 0.5;
    private static final double VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_LEAVE =
            VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_IN_POSITION
            + 4 * 0.5;
    /* the integral of the voltage of the intake ever since the note touches the lower gate */
    private double voltageIntegralSinceNoteInIntake;

    private double intakeVoltage = 0.0;
    private boolean shooterRunning = false;
    public IntakeIOSim() {
        super(
                new Translation2d(-DriveTrainConstants.BUMPER_LENGTH_METERS/2, 0.3),
                new Translation2d(-DriveTrainConstants.BUMPER_LENGTH_METERS/2, -0.3),
                1
        );
        // initial preload
        super.gamePieceCount = 1;
        voltageIntegralSinceNoteInIntake = VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_IN_POSITION;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        if (shooterRunning && voltageIntegralSinceNoteInIntake > VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_LEAVE) {
            super.gamePieceCount--;
            voltageIntegralSinceNoteInIntake = 0;
        }
        final boolean noteTouchingIntake = super.gamePieceCount > 0;
        inputs.lowerBeamBreakBlocked = noteTouchingIntake;
        if (noteTouchingIntake)
            voltageIntegralSinceNoteInIntake += intakeVoltage * Robot.defaultPeriodSecs;
        inputs.upperBeamBreakerBlocked = inputs.lowerBeamBreakBlocked
                && voltageIntegralSinceNoteInIntake >= VOLTAGE_INTEGRAL_SINCE_NOTE_IN_INTAKE_WHEN_NOTE_IN_POSITION;
    }

    @Override
    public void runIntakeVoltage(double volts) {
        this.intakeVoltage = volts;
    }

    @Override
    protected boolean isIntakeRunning() {
        return intakeVoltage > 5;
    }

    public void setShooterRunning(boolean shooterRunning) {
        this.shooterRunning = shooterRunning;
    }
}
