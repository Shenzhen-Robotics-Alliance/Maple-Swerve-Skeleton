package frc.robot.utils.CustomPIDs;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

public class MaplePIDController extends PIDController {
    private final MaplePIDConfig config;

    public MaplePIDController(MaplePIDConfig config) {
        super(config.Kp, config.Ki, config.Kd);
        if (config.isCircularLoop) super.enableContinuousInput(0, Units.rotationsToRadians(1));
        super.setTolerance(config.errorTolerance);
        this.config = config;
    }

    @Override
    public double calculate(double measurement) {
        final double constrainedCorrection =
                MathUtil.clamp(super.calculate(measurement), -config.maximumPower, config.maximumPower);

        if (Math.abs(constrainedCorrection) < config.deadBand) return 0;
        return constrainedCorrection;
    }

    public static final class MaplePIDConfig {
        public final double maximumPower, errorStartDecelerate, deadBand, errorTolerance, timeThinkAhead;
        public final double Kp, Ki, Kd;
        public final boolean isCircularLoop;

        public MaplePIDConfig(
                double maximumPower,
                double errorStartDecelerate,
                double percentDeadBand,
                double errorTolerance,
                double timeThinkAhead,
                boolean isCircularLoop,
                double ki) {
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
