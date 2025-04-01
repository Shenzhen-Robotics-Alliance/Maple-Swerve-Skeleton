package frc.robot.utils;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDAnimation;
import frc.robot.subsystems.led.LEDStatusLight;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AlertsManager {
    private static final List<Alert> instances = new ArrayList<>();

    public static Alert create(String text, Alert.AlertType type) {
        Alert alert = new Alert(text, type);
        register(alert);
        return alert;
    }

    public static void register(Alert alert) {
        instances.add(alert);
    }

    private static boolean hasAlertType(Alert.AlertType type) {
        return instances.stream().filter(Alert::get).anyMatch(alert -> alert.getType() == type);
    }

    public static void updateLEDAndLog(LEDStatusLight statusLight) {
        // Log alerts.
        Logger.recordOutput(
                "ActiveAlerts",
                instances.stream()
                        .filter(Alert::get)
                        .map(alert -> getPrefix(alert.getType()) + alert.getText())
                        .toArray(String[]::new));

        // Flash LED red if there is an Error.
        if (hasAlertType(Alert.AlertType.kError)) runIfNotRunning(getFlashLEDForErrorCommand(statusLight));
        // Flash LED Orange if there is a Warning.
        else if (hasAlertType(Alert.AlertType.kWarning) && DriverStation.isDisabled())
            runIfNotRunning(getFlashLEDForWarningCommand(statusLight));
    }

    private static Command flashLEDForError = null;
    /**
     * Creates the command the flashes the LED Red to show that there is an error. Errors are persistent, the LED will
     * not stop flashing until all errors are gone.
     */
    private static Command getFlashLEDForErrorCommand(LEDStatusLight statusLight) {
        if (flashLEDForError == null)
            flashLEDForError = statusLight
                    .playAnimationPeriodically(new LEDAnimation.Breathe(() -> Color.kRed), 2)
                    .until(() -> !hasAlertType(Alert.AlertType.kError))
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                    .ignoringDisable(true);
        return flashLEDForError;
    }

    private static Command flashLEDForWarning = null;
    /**
     * Creates the command the flashes the LED Orange to show that there is a warning. Warnings are not persistent, they
     * are gone after the robot is enabled. This command ends automatically if an Error is present, so that the LED
     * would instead be linking Red.
     */
    private static Command getFlashLEDForWarningCommand(LEDStatusLight statusLight) {
        if (flashLEDForWarning == null)
            flashLEDForWarning = statusLight
                    .playAnimationPeriodically(new LEDAnimation.Breathe(() -> Color.kOrange), 0.5)
                    .until(() -> !hasAlertType(Alert.AlertType.kWarning))
                    .until(() -> hasAlertType(Alert.AlertType.kError))
                    .until(DriverStation::isEnabled)
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                    .ignoringDisable(true);
        return flashLEDForWarning;
    }

    private static void runIfNotRunning(Command command) {
        if (!command.isScheduled()) command.schedule();
    }

    private static String getPrefix(Alert.AlertType type) {
        return switch (type) {
            case kError -> "Error: ";
            case kWarning -> "Warning: ";
            case kInfo -> "Info: ";
        };
    }
}
