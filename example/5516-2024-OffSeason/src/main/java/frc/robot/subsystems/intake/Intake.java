package frc.robot.subsystems.intake;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.JoystickDrive;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.subsystems.led.LEDStatusLight;
import frc.robot.subsystems.shooter.ShooterVisualizer;
import frc.robot.utils.Alert;
import frc.robot.utils.LEDAnimation;
import org.littletonrobotics.junction.Logger;

public class Intake extends MapleSubsystem {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs;

    private static final LEDAnimation RUNNING = new LEDAnimation.Charging(255, 255, 255, 2), // orange charging
            GRABBED_NOTE = new LEDAnimation.ShowColor(230, 255, 0); // yellow

    private boolean lowerBeamBrakeAlwaysTrue, upperBeamBrakeAlwaysTrue;
    private final Alert lowerBeamBrakeAlwaysBlockedAlert, upperBeamBrakeAlwaysBlockedAlert;
    private final BooleanConsumer noteInShooterConsumer;
    public Intake(IntakeIO intakeIO, BooleanConsumer noteInShooterConsumer) {
        super("Intake");
        this.io = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();

        this.lowerBeamBrakeAlwaysTrue = this.upperBeamBrakeAlwaysTrue = true;
        this.lowerBeamBrakeAlwaysBlockedAlert = new Alert("Intake LOWER Beam Breaker Always Blocked", Alert.AlertType.WARNING);
        this.upperBeamBrakeAlwaysBlockedAlert = new Alert("Intake UPPER Beam Breaker Always Blocked", Alert.AlertType.WARNING);

        this.noteInShooterConsumer = noteInShooterConsumer;

        super.setDefaultCommand(Commands.run(this::runIdle, this));
    }

    @Override
    public void onDisable() {
        runIdle();
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        this.lowerBeamBrakeAlwaysBlockedAlert.setActivated(
                lowerBeamBrakeAlwaysTrue &= inputs.lowerBeamBreakBlocked
        );
        this.upperBeamBrakeAlwaysBlockedAlert.setActivated(
                upperBeamBrakeAlwaysTrue &= inputs.upperBeamBreakerBlocked
        );
        noteInShooterConsumer.accept(isNotePresent());

        visualizeNoteInShooter();
    }

    private void visualizeNoteInShooter() {
        final ShooterVisualizer.NotePositionInShooter notePositionInShooter;
        if (inputs.upperBeamBreakerBlocked)
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.AT_TOP;
        else if (inputs.lowerBeamBreakBlocked)
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.AT_BOTTOM;
        else
            notePositionInShooter = ShooterVisualizer.NotePositionInShooter.GONE;
        ShooterVisualizer.setNoteInShooter(notePositionInShooter);
    }

    public void runFullIntakeVoltage() {
        io.runIntakeVoltage(12);
    }

    public void runInvertVoltage() {
        io.runIntakeVoltage(-12);
    }

    public void runMinimumPropellingVoltage() {
        io.runIntakeVoltage(4);
    }

    public boolean isNotePresent() {
        return inputs.upperBeamBreakerBlocked || inputs.lowerBeamBreakBlocked;
    }

    public boolean isNoteInUpperPosition() {
        return inputs.upperBeamBreakerBlocked;
    }

    public boolean isNoteTouchingIntake() {
        return inputs.lowerBeamBreakBlocked;
    }

    public Command suckNoteUntilTouching() {
        return Commands.run(this::runFullIntakeVoltage, this)
                .onlyIf(() -> !inputs.lowerBeamBreakBlocked)
                .until(() -> inputs.lowerBeamBreakBlocked)
                .finallyDo(this::runMinimumPropellingVoltage);
    }

    public Command executeIntakeNote() {
        return Commands.run(() -> {
                    if (inputs.lowerBeamBreakBlocked)
                        runMinimumPropellingVoltage();
                    else runFullIntakeVoltage();
                }, this)
                .until(() -> inputs.upperBeamBreakerBlocked)
                .onlyIf(() -> !inputs.upperBeamBreakerBlocked)
                .finallyDo(this::runIdle);
    }

    public Command executeIntakeNote(LEDStatusLight statusLight, XboxController xboxController) {
        return executeIntakeNote()
                .raceWith(Commands.run(() -> statusLight.setAnimation(RUNNING), statusLight))
                .andThen(statusLight.playAnimationAndStop(GRABBED_NOTE, 1.5)
                        .deadlineWith(rumbleGamepad(xboxController))
                );
    }

    public static Command rumbleGamepad(XboxController xboxController) {
        return Commands.runEnd(
                () -> xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 1),
                () -> xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0)
        );
    }

    public Command executeLaunch() {
        return Commands.run(this::runFullIntakeVoltage, this)
                .onlyIf(this::isNotePresent)
                .until(() -> !isNotePresent())
                .finallyDo(this::runIdle);
    }

    public void runIdle() {
        io.runIntakeVoltage(0);
    }
}

