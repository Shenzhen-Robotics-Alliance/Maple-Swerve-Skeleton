package frc.robot.utils.Math; // package frc.robot.Helpers.MathHelpers;
//
/// **
// * Bézier curve with four points
// */
//
/// **
// * Bézier curve with four points
// */
// public class BezierCurve {
//    private static final int resolution = 20;
//    public final double length;
//    private final Vector2D p0, p1, p2, p3;
//    public final Vector2D[] previewPoints = new Vector2D[resolution];
//
//    /**
//     * create a bezier curve to be a straight line
//     * */
//    public BezierCurve(Vector2D startingPoint, Vector2D endingPoint) {
//        this(
//                startingPoint,
//                startingPoint.addBy(Vector2D.displacementToTarget(startingPoint,
// endingPoint).multiplyBy(1.0/3.0)),
//                startingPoint.addBy(Vector2D.displacementToTarget(startingPoint,
// endingPoint).multiplyBy(2.0/3.0)),
//                endingPoint);
//    }
//    public BezierCurve(Vector2D startingPoint, Vector2D midPoint, Vector2D endingPoint) {
//        this(startingPoint, midPoint, midPoint, endingPoint);
//    }
//    public BezierCurve(Vector2D startingPoint, Vector2D startingPointAnotherPoint, Vector2D
// endingPointAnotherPoint, Vector2D endingPoint) {
//        this.p0 = startingPoint;
//        this.p1 = startingPointAnotherPoint;
//        this.p2 = endingPointAnotherPoint;
//        this.p3 = endingPoint;
//
//        for (int i = 0; i < resolution; i++)
//            previewPoints[i] = getPositionWithLERP(1.0 * i / resolution);
//        this.length = calculateLength(resolution);
//    }
//
//    public BezierCurve getReversedCurve() {
//        return new BezierCurve(p3, p2, p1, p0);
//    }
//
//    public Vector2D getPositionWithLERP(double t) {
//        if (t<0) t=0;
//        if (t>1) t=1;
//        /*
//         * A = LERP(p0,p1,t)
//         * B = LERP(p1,p2,t)
//         * C = LERP(p2,p3,t)
//         * D = LERP(A,B,t)
//         * E = LERP(B,C,t)
//         * P(t) = LERP(D,E,t) =
//         *   p0(-t^3 + 3t^2 - 3t + 1) +
//         *   p1(3t^3 -6t^2 + 3t) +
//         *   p2(-3t^3 + 3t^2) +
//         *   p3(t^3)
//         */
//        Vector2D position = new Vector2D();
//        position = position.addBy(p0.multiplyBy(-Math.pow(t,3) + 3*t*t - 3*t +1));
//        position = position.addBy(p1.multiplyBy(3 * Math.pow(t,3) - 6*t*t + 3*t));
//        position = position.addBy(p2.multiplyBy(-3 * Math.pow(t,3) + 3*t*t));
//        position = position.addBy(p3.multiplyBy(Math.pow(t,3)));
//
//        return position;
//    }
//
//    public Vector2D getVelocityWithLERP(double t) {
//        /*
//         * P'(t) = LERP'(D,E,t) =
//         *   p0(-3t^2 + 6t - 3) +
//         *   p1(9t^2 -12t + 3) +
//         *   p2(-9t^2 + 6t) +
//         *   p3(3t^2)
//         */
//        Vector2D velocity = new Vector2D();
//        velocity = velocity.addBy(p0.multiplyBy(-3*t*t + 6*t - 3));
//        velocity = velocity.addBy(p1.multiplyBy(9*t*t - 12 *t + 3));
//        velocity = velocity.addBy(p2.multiplyBy(-9*t*t + 6*t));
//        velocity = velocity.addBy(p3.multiplyBy(3*t*t));
//
//        return velocity;
//    }
//
//    public Vector2D getAccelerationWithLERP(double t) {
//        /*
//         * P''(t) = LERP''(D,E,t) =
//         *   p0(-6t + 6) +
//         *   p1(18t-12) +
//         *   p2(-18t + 6) +
//         *   p3(6t)
//         */
//        Vector2D acceleration = new Vector2D();
//        acceleration = acceleration.addBy(p0.multiplyBy(-6*t + 6));
//        acceleration = acceleration.addBy(p1.multiplyBy(18*t - 12));
//        acceleration = acceleration.addBy(p2.multiplyBy(-18*t + 6));
//        acceleration = acceleration.addBy(p3.multiplyBy(6*t));
//
//        return acceleration;
//    }
//
//    // TODO get the boundary box
//
//    public double calculateLength(int samples) {
//        double length = 0;
//        for (double t = 0; t <= 1; t += 1.0d/samples)
//            length += getVelocityWithLERP(t).getMagnitude() / samples;
//        return length;
//    }
// }
