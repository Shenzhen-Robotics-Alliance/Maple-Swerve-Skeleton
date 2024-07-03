package frc.robot.utils.PhysicsSimulation.FieldMaps; // package
                                                     // frc.robot.Helpers.PhysicsSimulation.FieldMaps;
//
// import frc.robot.Utils.MathUtils.Rotation2D;
// import frc.robot.Utils.MathUtils.Vector2D;
// import frc.robot.Utils.PhysicsSimulation.AllRealFieldPhysicsSimulation;
//
// public class CrescendoDefault extends AllRealFieldPhysicsSimulation.FieldCollisionMap {
//    public CrescendoDefault() {
//        super();
//        // lefter wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0, 1}),
//                new Vector2D(new double[] {0, 4.51})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0, 4.51}),
//                new Vector2D(new double[] {0.9, 5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0.9, 5}),
//                new Vector2D(new double[] {0.9, 6.05})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0.9, 6.05}),
//                new Vector2D(new double[] {0, 6.5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0.9, 6.05}),
//                new Vector2D(new double[] {0, 6.5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0, 6.5}),
//                new Vector2D(new double[] {0, 8.2})
//        );
//
//        // upper wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {0, 8.12}),
//                new Vector2D(new double[] {16.54, 8.12})
//        );
//
//        // righter wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54, 1}),
//                new Vector2D(new double[] {16.54, 4.51})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54, 4.51}),
//                new Vector2D(new double[] {16.54-0.9, 5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54-0.9, 5}),
//                new Vector2D(new double[] {16.54-0.9, 6.05})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54-0.9, 6.05}),
//                new Vector2D(new double[] {16.54, 6.5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54-0.9, 6.05}),
//                new Vector2D(new double[] {16.54, 6.5})
//        );
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54, 6.5}),
//                new Vector2D(new double[] {16.54, 8.2})
//        );
//
//        // lower wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {1.92, 0}),
//                new Vector2D(new double[] {16.54-1.92, 0})
//        );
//
//        // red source wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {1.92, 0}),
//                new Vector2D(new double[] {0, 1})
//        );
//
//        // blue source wall
//        super.addLinearObstacle(
//                new Vector2D(new double[] {16.54-1.92, 0}),
//                new Vector2D(new double[] {16.54, 1})
//        );
//
//        // blue stage
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {3.4, 4.1}),
//                0.35, 0.35,
//                new Rotation2D(0)
//        );
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {5.62, 4.1-1.28}),
//                0.35, 0.35,
//                new Rotation2D(Math.toRadians(30))
//        );
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {5.62, 4.1+1.28}),
//                0.35, 0.35,
//                new Rotation2D(Math.toRadians(60))
//        );
//
//        // red stage
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {16.54-3.4, 4.1}),
//                0.35, 0.35,
//                new Rotation2D(0)
//        );
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {16.54-5.62, 4.1-1.28}),
//                0.35, 0.35,
//                new Rotation2D(Math.toRadians(60))
//        );
//        super.addRectangularObstacle(
//                new Vector2D(new double[] {16.54-5.62, 4.1+1.28}),
//                0.35, 0.35,
//                new Rotation2D(Math.toRadians(30))
//        );
//    }
// }
