package frc.robot.utils.MechanismControlHelpers; // package
                                                 // frc.robot.Helpers.MechanismControlHelpers;
//
// import frc.robot.Helpers.MathHelpers.AngleUtils;
// import frc.robot.Helpers.MathHelpers.LookUpTable;
//
/// **
// * This is an easy tool to control mechanisms with the most basic PID algorithm.
// * This class has no essential different from PIDController class by WPILIB, except that this
// class automatically calculates the kP and kD according to some parameters
// * */
// public class EasyPIDController implements SingleDimensionMechanismController {
//    /** the profile of the mechanism being controlled */
//    private final EasyPIDProfile easyPIDProfile;
//
//    private double desiredPosition;
//    /**
//     * initializes an easy pid controller with a given profile
//     * */
//    public EasyPIDController(EasyPIDProfile easyPIDProfile, double startingPosition) {
//        this.easyPIDProfile = easyPIDProfile;
//        desiredPosition = startingPosition;
//    }
//
//    @Override
//    public double getMotorPower(double mechanismVelocity, double mechanismPosition) {
//        final double
//                mechanismPositionWithFeedForward = mechanismPosition + mechanismVelocity *
// easyPIDProfile.mechanismDecelerationTime,
//                error = easyPIDProfile.isMechanismInCycle ?
//                        AngleUtils.getActualDifference(mechanismPositionWithFeedForward,
// desiredPosition)
//                        : desiredPosition - mechanismPositionWithFeedForward;
//        if (Math.abs(error) < easyPIDProfile.errorTolerance)
//            return 0;
//        final double power =
// LookUpTable.linearInterpretationWithBounding(easyPIDProfile.errorTolerance,
// easyPIDProfile.minimumPower, easyPIDProfile.errorStartDecelerate, easyPIDProfile.maximumPower,
// Math.abs(error));
//        return Math.copySign(power, error);
//    }
//
//    public void setDesiredPosition(double desiredPosition) {
//        this.desiredPosition = desiredPosition;
//    }
//
//    public static final class EasyPIDProfile {
//        private final double maximumPower, errorStartDecelerate, minimumPower, errorTolerance,
// mechanismDecelerationTime;
//        private final boolean isMechanismInCycle;
//        public EasyPIDProfile(double maximumPower, double errorStartDecelerate, double
// minimumPower, double errorTolerance, double mechanismDecelerationTime, boolean
// isMechanismInCycle) {
//            this.maximumPower = maximumPower;
//            this.errorStartDecelerate = errorStartDecelerate;
//            this.minimumPower = minimumPower;
//            this.errorTolerance = errorTolerance;
//            this.mechanismDecelerationTime = mechanismDecelerationTime;
//            this.isMechanismInCycle = isMechanismInCycle;
//        }
//    }
// }
