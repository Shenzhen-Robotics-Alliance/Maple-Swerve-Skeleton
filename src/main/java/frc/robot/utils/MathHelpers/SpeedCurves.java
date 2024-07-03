package frc.robot.utils.MathHelpers; // package frc.robot.Helpers.MathHelpers;
//
/// *
// * https://developer.mozilla.org/en-US/docs/Web/CSS/easing-function
// *  */
// public class SpeedCurves {
//    public interface SpeedCurve {
//        /**
//         * @param originalT: the original t value, 0~1
//         * @return the scaled t, 0~1
//         * */
//        double getScaledT(double originalT);
//    }
//
//    public static final SpeedCurve originalSpeed = (originalT) -> originalT;
//
//    public static final class CubicBezierSpeedCurve extends LookUpTable implements SpeedCurve {
//        private static final int samples = 10;
//        private static double[] getXValues(BezierCurve curve) {
//            double[] ans = new double[samples];
//            for (int sample = 0; sample < samples; sample++)
//                ans[sample] = curve.getPositionWithLERP(1.0 * sample / samples).getX();
//            return ans;
//        }
//
//        private static double[] getYValues(BezierCurve curve) {
//            double[] ans = new double[samples];
//            for (int sample = 0; sample < samples; sample++)
//                ans[sample] = curve.getPositionWithLERP(1.0 * sample / samples).getY();
//            return ans;
//        }
//
//        private static BezierCurve getBezierCurve(double x1, double y1, double x2, double y2) {
//            return new BezierCurve(
//                    new Vector2D(new double[] {0, 0}),
//                    new Vector2D(new double[] {x1, y1}),
//                    new Vector2D(new double[] {x2, y2}),
//                    new Vector2D(new double[] {1, 1}));
//        }
//        public CubicBezierSpeedCurve(double x1, double y1, double x2, double y2) {
//            super(getXValues(getBezierCurve(x1, y1, x2, y2)), getYValues(getBezierCurve(x1, y1,
// x2, y2)));
//        }
//
//        @Override
//        public double getScaledT(double originalT) {
//            if (originalT <= 0)
//                return 0;
//            else if (originalT >= 1)
//                return 1;
//            return super.getYPrediction(originalT);
//        }
//    }
//
//    public static final CubicBezierSpeedCurve
//            slowDown = new CubicBezierSpeedCurve(0.1, 0.6, 0.7, 0.2),
//            easeIn = new CubicBezierSpeedCurve(0.42, 0, 1, 1),
//            easeOut = new CubicBezierSpeedCurve(0, 0, 0.58, 1),
//            easeInOut = new CubicBezierSpeedCurve(0.42, 0, 0.58, 1);
// }
