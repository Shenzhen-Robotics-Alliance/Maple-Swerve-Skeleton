package frc.robot.utils.CustomMaths;

public class Angles {
    /**
     * simplify an angle into the range 0-360 degrees
     *
     * @param radian the angle to simplify, in radian
     * @return the simplified angle, in radian and in the range 0 < x < Math.Pi*2
     */
    public static double simplifyAngle(double radian) {
        if (Double.isNaN(radian) || Double.isInfinite(radian) || Math.abs(radian) > 10e7)
            throw new IllegalArgumentException("invalid radian: " + radian);
        radian = Math.copySign(radian % (Math.PI * 2), radian);
        if (radian < 0) radian += Math.PI * 2;
        return radian;
    }

    /**
     * gets the shortest rotational distance(and its direction) needed to get from the current to
     * targeted rotation
     *
     * @param currentRotation  the current rotation, in radian
     * @param targetedRotation the desired rotation, in radian
     * @return the shortest distance between the two points, in radian and positive is
     * counter-clockwise
     */
    public static double getActualDifference(double currentRotation, double targetedRotation) {
        final double loopLength = Math.PI * 2;
        currentRotation = simplifyAngle(currentRotation);
        targetedRotation = simplifyAngle(targetedRotation);
        double difference = targetedRotation - currentRotation;
        if (difference > loopLength / 2) return -(loopLength - difference); // go the other way around
        if (difference < -loopLength / 2) return loopLength + difference; // go the other way around
        return difference;
    }

    /**
     * get the mid point between two points
     */
    public static double findMidPoint(double rotation1, double rotation2) {
        return simplifyAngle(rotation1 + getActualDifference(rotation1, rotation2));
    }
}
