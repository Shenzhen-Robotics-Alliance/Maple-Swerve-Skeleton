package frc.robot.utils.MechanismControl; // package
                                                 // frc.robot.Helpers.MechanismControlHelpers;
//
// import frc.robot.Helpers.MathHelpers.Vector2D;
//
// public class ChassisPositionController {
//    private final ChassisPIDConfig config;
//    private Vector2D desiredPosition2D = null;
//    private long previousTimeNano;
//    private static final double nanoToSeconds = 10e-9;
//
//    public ChassisPositionController(ChassisPIDConfig config) {
//        this.config = config;
//    }
//
//    public void setDesiredPosition(Vector2D desiredPosition) {
//        this.desiredPosition2D = desiredPosition;
//        this.previousTimeNano = System.nanoTime();
//        this.previousPosition2D = null;
//    }
//
//    private Vector2D previousPosition2D;
//    public Vector2D getCorrectionPower(Vector2D currentPosition2D) {
//        final double dt = (System.nanoTime() - previousTimeNano) * nanoToSeconds;
//        System.out.println("position controller dt: " + dt);
//        if (previousPosition2D == null)
//            previousPosition2D = currentPosition2D;
//        final Vector2D positionDifference =
// currentPosition2D.addBy(previousPosition2D.multiplyBy(-1));
//
//        previousTimeNano = System.nanoTime();
//        previousPosition2D = currentPosition2D;
//
//        return getCorrectionPower(currentPosition2D, positionDifference.multiplyBy(1/dt));
//    }
//
//    public Vector2D getCorrectionPower(Vector2D currentPosition2D, Vector2D currentVelocity2D) {
//        final Vector2D processedPosition =
// currentPosition2D.addBy(currentVelocity2D.multiplyBy(config.robotPositionFeedForwardTime)),
//                positionDifference = desiredPosition2D.addBy(processedPosition.multiplyBy(-1));
//        final double positionDifferenceMagnitude = positionDifference.getMagnitude(),
//                correctionPowerMagnitude =
// config.getCorrectionPowerMagnitude(positionDifferenceMagnitude);
//
//        return new Vector2D(positionDifference.getHeading(), correctionPowerMagnitude);
//    }
//
//    public static class ChassisPIDConfig {
//        public final double positionErrorTolerance;
//        public final double positionErrorStartDecelerate;
//        public final double positionMaximumCorrectionPower;
//        public final double positionMinimumCorrectionPower;
//        public final double robotPositionFeedForwardTime;
//
//        /**
//         * creates a chassis PID config
//         * @param positionErrorTolerance
//         * @param positionErrorStartDecelerate
//         * @param positionMaximumCorrectionPower
//         * @param positionMinimumCorrectionPower
//         * @param robotPositionFeedForwardTime
//         */
//        public ChassisPIDConfig(double positionMaximumCorrectionPower, double
// positionMinimumCorrectionPower, double positionErrorStartDecelerate, double
// positionErrorTolerance,  double robotPositionFeedForwardTime) {
//            this.positionErrorTolerance = positionErrorTolerance;
//            this.positionErrorStartDecelerate = positionErrorStartDecelerate;
//            this.positionMaximumCorrectionPower = positionMaximumCorrectionPower;
//            this.positionMinimumCorrectionPower = positionMinimumCorrectionPower;
//            this.robotPositionFeedForwardTime = robotPositionFeedForwardTime;
//        }
//
//        public double getCorrectionPowerMagnitude(double positionDifference) {
//            double correctionPowerMagnitude = 0;
//            final double proportionGain = (positionMaximumCorrectionPower -
// positionMinimumCorrectionPower) / (positionErrorStartDecelerate - positionErrorTolerance);
//            if (positionDifference > positionErrorTolerance)
//                correctionPowerMagnitude = (positionDifference - positionErrorTolerance) *
// proportionGain + positionMinimumCorrectionPower;
//            return Math.min(correctionPowerMagnitude, positionMaximumCorrectionPower);
//        }
//    }
// }
