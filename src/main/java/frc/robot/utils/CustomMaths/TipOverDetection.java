package frc.robot.utils.CustomMaths;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class TipOverDetection {
    public static double getTippingAngleRad(Rotation3d driveTrain3dOrientation) {
        Translation3d unitVerticalVector = new Translation3d(0, 0, 1).rotateBy(driveTrain3dOrientation);
        double horizontalComponent = unitVerticalVector.toTranslation2d().getNorm();
        double verticalComponent = unitVerticalVector.getZ();
        double normalLineAngleRad = Math.atan2(verticalComponent, horizontalComponent);
        return Math.abs(normalLineAngleRad - Math.toRadians(90));
    }
}
