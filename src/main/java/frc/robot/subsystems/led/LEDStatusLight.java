package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends MapleSubsystem {
    private static final int DASHBOARD_DISPLAY_LENGTH = 32;
    private static AddressableLED led = null;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer bufferForDashboard;

    public LEDStatusLight(int port, int length) {
        super("LED");
        this.buffer = new AddressableLEDBuffer(length);
        this.bufferForDashboard = new AddressableLEDBuffer(DASHBOARD_DISPLAY_LENGTH);

        if (led != null) led.close();
        led = new AddressableLED(port);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    final String[] dashboardColors = new String[DASHBOARD_DISPLAY_LENGTH];

    @Override
    public void periodic(double dt, boolean enabled) {
        for (int i = 0; i < DASHBOARD_DISPLAY_LENGTH; i++)
            dashboardColors[i] = bufferForDashboard.getLED(i).toHexString();

        led.setData(buffer);
        if (!Robot.isReal()) Logger.recordOutput("Status Light", dashboardColors);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds) {
        Timer timer = new Timer();
        timer.start();
        return this.run(() -> {
                    double t = timer.get() / timeSeconds;
                    animation.play(buffer, t);
                    animation.play(bufferForDashboard, t);
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

    public Command showEnableDisableState() {
        return new ConditionalCommand(
                        playAnimation(new LEDAnimation.SlideBackAndForth(0, 200, 255, 0.8), 0.8)
                                .until(RobotState::isDisabled),
                        playAnimation(new LEDAnimation.Breathe(0, 200, 255), 2).until(RobotState::isEnabled),
                        RobotState::isEnabled)
                .repeatedly()
                .ignoringDisable(true);
    }
}
