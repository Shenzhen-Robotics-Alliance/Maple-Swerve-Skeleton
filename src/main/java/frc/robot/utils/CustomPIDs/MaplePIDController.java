package frc.robot.utils.CustomPIDs;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class MaplePIDController extends PIDController {
    private final MaplePIDConfig config;
    public MaplePIDController(
            MaplePIDConfig config
    ) {
        super(config.Kp, config.Ki, config.Kd);
        if (config.isCircularLoop)
            super.enableContinuousInput(0, Units.rotationsToRadians(1));
        super.setTolerance(config.errorTolerance);
        this.config = config;
    }

    @Override
    public double calculate(double measurement) {
        return MathUtil.clamp(
                MathUtil.applyDeadband(super.calculate(measurement), config.deadBand)
                , -config.maximumPower, config.maximumPower);
    }

    public static final class MaplePIDConfig {
        final double maximumPower, errorStartDecelerate, deadBand, errorTolerance, timeThinkAhead;
        final double Kp, Ki, Kd;
        final boolean isCircularLoop;

        public MaplePIDConfig(double maximumPower, double errorStartDecelerate, double percentDeadBand, double errorTolerance, double timeThinkAhead, boolean isCircularLoop, double ki) {
            this.maximumPower = maximumPower;
            this.errorStartDecelerate = errorStartDecelerate;
            this.deadBand = percentDeadBand * maximumPower;
            this.errorTolerance = errorTolerance;
            this.timeThinkAhead = timeThinkAhead;
            this.isCircularLoop = isCircularLoop;
            this.Ki = ki;

            this.Kp = maximumPower / errorStartDecelerate;
            this.Kd = Kp * timeThinkAhead;
        }

        public PIDConstants toPathPlannerPIDConstants() {
            return new PIDConstants(this.Kp, this.Ki, this.Kd);
        }
    }
}
