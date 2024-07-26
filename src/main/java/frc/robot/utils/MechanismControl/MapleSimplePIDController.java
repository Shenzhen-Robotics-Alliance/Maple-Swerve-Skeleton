package frc.robot.utils.MechanismControl;

import frc.robot.utils.MapleMaths.Angles;
import frc.robot.utils.MapleMaths.MapleCommonMath;
import org.littletonrobotics.junction.Logger;

/**
 * This is an easy tool to control mechanisms with the most basic PID algorithm. This class has no
 * essential different from PIDController class by WPILIB, except that this class automatically
 * calculates the kP and kD according to some parameters
 */
public class MapleSimplePIDController implements SingleDimensionMechanismController {
    /**
     * the profile of the mechanism being controlled
     */
    private final SimplePIDConfig profile;

    private double desiredPosition;

    /**
     * initializes an easy pid controller with a given config
     */
    public MapleSimplePIDController(SimplePIDConfig config, double startingPosition) {
        this.profile = config;
        desiredPosition = startingPosition;
    }

    @Override
    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
        final double
                mechanismPositionWithDerivative =
                mechanismPosition + mechanismVelocity * profile.mechanismDecelerationTime,
                error =
                        profile.isMechanismInCycle
                                ? Angles.getActualDifference(mechanismPositionWithDerivative, desiredPosition)
                                : desiredPosition - mechanismPositionWithDerivative;
        if (Math.abs(error) < profile.errorTolerance) return 0;
        final double
                unsignedPower =
                MapleCommonMath.linearInterpretationWithBounding(
                        profile.errorTolerance,
                        profile.minimumPower,
                        profile.errorStartDecelerate,
                        profile.maximumPower,
                        Math.abs(error)),
                correctionPower = Math.copySign(unsignedPower, error);
        Logger.recordOutput(
                "/ClosedLoops/MapleSimplePID/currentPositionWithDerivative",
                mechanismPositionWithDerivative);
        Logger.recordOutput("/ClosedLoops/MapleSimplePID/desiredPosition", desiredPosition);
        Logger.recordOutput("/ClosedLoops/MapleSimplePID/error", error);
        Logger.recordOutput("/ClosedLoops/MapleSimplePID/power", correctionPower);
        return correctionPower;
    }

    public void setDesiredPosition(double desiredPosition) {
        this.desiredPosition = desiredPosition;
    }

    public static final class SimplePIDConfig {
        private final double maximumPower,
                errorStartDecelerate,
                minimumPower,
                errorTolerance,
                mechanismDecelerationTime;
        private final boolean isMechanismInCycle;

        public SimplePIDConfig(
                double maximumPower,
                double errorStartDecelerate,
                double minimumPower,
                double errorTolerance,
                double mechanismDecelerationTime,
                boolean isMechanismInCycle) {
            this.maximumPower = maximumPower;
            this.errorStartDecelerate = errorStartDecelerate;
            this.minimumPower = minimumPower;
            this.errorTolerance = errorTolerance;
            this.mechanismDecelerationTime = mechanismDecelerationTime;
            this.isMechanismInCycle = isMechanismInCycle;
        }
    }
}
