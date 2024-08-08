package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.MapleSubsystem;
import frc.robot.utils.LEDAnimation;
import org.littletonrobotics.junction.Logger;

public class LEDStatusLight extends MapleSubsystem {
    private static final int DASHBOARD_DISPLAY_LENGTH = 41;
    private static AddressableLED led = null;
    private final AddressableLEDBuffer buffer;
    private final AddressableLEDBuffer bufferForDashboard;
    private final Timer t = new Timer();

    private LEDAnimation animation;
    public LEDStatusLight(int port, int length) {
        super("LED");
        if (led != null) led.close();
        led = Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REAL ? new AddressableLED(port) : null;
        if (led != null) led.setLength(length);

        this.buffer = new AddressableLEDBuffer(length);
        this.bufferForDashboard = new AddressableLEDBuffer(DASHBOARD_DISPLAY_LENGTH);

        super.setDefaultCommand(playAnimation(new LEDAnimation.ShowColor(0, 0, 0)));
    }

    @Override
    public void onReset() {
        t.start();
        if (led != null) led.start();
        this.animation = new LEDAnimation.ShowColor(0, 0, 0); // turn off
    }


    @Override
    public void periodic(double dt, boolean enabled) {}

    final String[] colors = new String[DASHBOARD_DISPLAY_LENGTH/2];
    private void updateAnimation(LEDAnimation animation) {
        animation.play(buffer, t.get());
        animation.play(bufferForDashboard, t.get());
        for (int i = 0; i < DASHBOARD_DISPLAY_LENGTH/2; i++)
            colors[i] = bufferForDashboard.getLED(DASHBOARD_DISPLAY_LENGTH/2 + i).toHexString();

        if (led != null) led.setData(buffer);
        if (Robot.CURRENT_ROBOT_MODE == Constants.RobotMode.REPLAY)
            Logger.recordOutput("Status Light", colors);
        else
            SmartDashboard.putStringArray("Status Light", colors);
    }

    public Command playAnimation(LEDAnimation animation) {
        return Commands.run(() -> updateAnimation(animation), this)
                .beforeStarting(t::reset);
    }

    public Command playAnimation(LEDAnimation animation, double timeSeconds) {
        return playAnimation(animation)
                .withTimeout(timeSeconds);
    }
}
