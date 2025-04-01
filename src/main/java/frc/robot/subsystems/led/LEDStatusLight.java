package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends SubsystemBase {
    private static final int DASHBOARD_DISPLAY_LENGTH = 8;
    private static AddressableLED led = null;
    private final Color[] ledColors;
    private final Color[] dashboardColors;
    private final String[] dashboardColorsHex;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView[] views;
    private final boolean[] viewsReversed;

    public LEDStatusLight(int port, int lengthOfEachSide, boolean... viewsReversed) {
        if (Robot.isSimulation()) Arrays.fill(viewsReversed, true);
        this.viewsReversed = viewsReversed;

        int length = lengthOfEachSide * viewsReversed.length;

        this.ledColors = new Color[lengthOfEachSide];
        this.dashboardColors = new Color[DASHBOARD_DISPLAY_LENGTH];
        this.dashboardColorsHex = new String[DASHBOARD_DISPLAY_LENGTH];
        Arrays.fill(ledColors, new Color());
        Arrays.fill(dashboardColors, new Color());
        this.buffer = new AddressableLEDBuffer(length);

        views = new AddressableLEDBufferView[viewsReversed.length];
        for (int i = 0; i < viewsReversed.length; i++)
            views[i] = buffer.createView(i * lengthOfEachSide, lengthOfEachSide);

        if (led != null) led.close();
        led = new AddressableLED(port);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic() {
        for (int led = 0; led < viewsReversed.length; led++)
            for (int i = 0; i < ledColors.length; i++)
                views[led].setLED(viewsReversed[led] ? ledColors.length - i - 1 : i, ledColors[i]);
        led.setData(buffer);
        for (int i = 0; i < DASHBOARD_DISPLAY_LENGTH; i++) dashboardColorsHex[i] = dashboardColors[i].toHexString();
        if (Robot.LOG_DETAILS) Logger.recordOutput("Status Light", dashboardColorsHex);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds) {
        Timer timer = new Timer();
        timer.start();
        return this.run(() -> {
                    animation.play(ledColors, timer.get() / timeSeconds);
                    animation.play(dashboardColors, timer.get() / timeSeconds);
                })
                .beforeStarting(timer::reset)
                .withTimeout(timeSeconds)
                .ignoringDisable(true);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds, int loopNum) {
        return playAnimation(animation, timeSeconds).repeatedly().withTimeout(timeSeconds * loopNum);
    }

    public Command playAnimationPeriodically(LEDAnimation animation, double hz) {
        double timeSeconds = 1.0 / hz;
        return this.playAnimation(animation, timeSeconds).repeatedly().ignoringDisable(true);
    }

    public Command showRobotState() {
        return defer(() -> {
                    if (DriverStation.isEnabled())
                        return playAnimation(
                                        new LEDAnimation.SlideBackAndForth(
                                                () -> RobotState.getInstance().visionObservationRate() > 0.25
                                                        ? new Color(0, 200, 255)
                                                        : new Color(255, 255, 255)),
                                        2.5)
                                .until(DriverStation::isDisabled);
                    return playAnimation(
                                    new LEDAnimation.Breathe(() -> RobotContainer.motorBrakeEnabled
                                            ? new Color(0, 200, 255)
                                            : new Color(255, 255, 255)),
                                    3)
                            .until(DriverStation::isEnabled);
                })
                .repeatedly()
                .ignoringDisable(true);
    }
}
