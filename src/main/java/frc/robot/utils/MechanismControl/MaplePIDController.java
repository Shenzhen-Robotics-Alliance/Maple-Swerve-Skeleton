package frc.robot.utils.MechanismControl;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class MaplePIDController extends PIDController {
    private final double maximumPower;
    public MaplePIDController(
            MaplePIDConfig config
    ) {
        super(config.Kp, config.Ki, config.Kd);
        if (config.isCircularLoop)
            super.enableContinuousInput(0, Units.rotationsToRadians(1));
        super.setTolerance(config.errorTolerance);
        this.maximumPower = config.maximumPower;
    }

    @Override
    public double calculate(double measurement) {
        return MathUtil.clamp(super.calculate(measurement), -maximumPower, maximumPower);
    }

    public static final class MaplePIDConfig {
        final double maximumPower, errorStartDecelerate, errorTolerance, timeThinkAhead;
        final double Kp, Ki, Kd;
        final boolean isCircularLoop;

        public MaplePIDConfig(double maximumPower, double errorStartDecelerate, double errorTolerance, double timeThinkAhead, boolean isCircularLoop, double ki) {
            this.maximumPower = maximumPower;
            this.errorStartDecelerate = errorStartDecelerate;
            this.errorTolerance = errorTolerance;
            this.timeThinkAhead = timeThinkAhead;
            this.isCircularLoop = isCircularLoop;
            this.Ki = ki;

            this.Kp = maximumPower / errorStartDecelerate;
            this.Kd = Kp * timeThinkAhead;
        }
    }
}
