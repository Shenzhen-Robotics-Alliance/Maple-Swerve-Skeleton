package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.MapleSubsystem;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends MapleSubsystem {
    private static AddressableLED led = null;
    private final Color[] ledColors;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBufferView view1, view2;

    public LEDStatusLight(int port, int length) {
        super("LED");
        // make sure length is even
        length = length / 2 * 2;
        this.ledColors = new Color[length / 2 - 1];
        Arrays.fill(ledColors, new Color());
        this.buffer = new AddressableLEDBuffer(length);

        view1 = buffer.createView(0, length / 2).reversed();
        view2 = buffer.createView(length / 2 + 1, length - 1);

        if (led != null) led.close();
        led = new AddressableLED(port);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        for (int i = 0; i < ledColors.length; i++) {
            view1.setLED(i, ledColors[i]);
            view2.setLED(i, ledColors[i]);
        }

        led.setData(buffer);
        Logger.recordOutput(
                "Status Light", Arrays.stream(ledColors).map(Color::toHexString).toArray(String[]::new));
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds) {
        Timer timer = new Timer();
        timer.start();
        return this.run(() -> animation.play(ledColors, timer.get() / timeSeconds))
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

    public Command showEnableDisableState() {
        return new ConditionalCommand(
                        playAnimation(new LEDAnimation.SlideBackAndForth(new Color(0, 200, 255)), 5)
                                .until(RobotState::isDisabled),
                        playAnimation(new LEDAnimation.Breathe(new Color(0, 200, 255)), 3)
                                .until(RobotState::isEnabled),
                        RobotState::isEnabled)
                .repeatedly()
                .ignoringDisable(true);
    }
}
