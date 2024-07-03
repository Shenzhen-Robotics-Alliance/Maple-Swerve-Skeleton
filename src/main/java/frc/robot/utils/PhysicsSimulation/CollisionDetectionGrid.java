package frc.robot.utils.PhysicsSimulation; // package frc.robot.Helpers.PhysicsSimulation;
//
// import edu.wpi.first.wpilibj.Filesystem;
// import frc.robot.Utils.EasyDataFlow;
// import frc.robot.Utils.MathUtils.Vector2D;
// import org.json.simple.JSONArray;
// import org.json.simple.JSONObject;
// import org.json.simple.parser.JSONParser;
//
// import java.io.BufferedReader;
// import java.io.File;
// import java.io.FileReader;
//
// public class CollisionDetectionGrid {
//    private static final double bounceCoefficient = -0.1, bounceBackDistance = 0.05,
// maxImpactVelocity = 5;
//    private boolean[][] grid = null;
//    private double cellSize = 0, fieldLength = 0, fieldWidth = 0;
//
//    public CollisionDetectionGrid() {
//        try {
//            File navGridFile = new File(Filesystem.getDeployDirectory(),
// "pathplanner/navgrid.json");
//            StringBuilder fileContentBuilder = new StringBuilder();
//            String line;
//            BufferedReader br = new BufferedReader(new FileReader(navGridFile));
//            while ((line = br.readLine()) != null) {
//                fileContentBuilder.append(line);
//            }
//
//            String fileContent = fileContentBuilder.toString();
//            JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
//
//            cellSize = ((Number) json.get("nodeSizeMeters")).doubleValue();
//            JSONArray grid = (JSONArray) json.get("grid");
//            this.grid = new boolean[grid.size()][((JSONArray) grid.get(0)).size()];
//
//            for (int row = 0; row < grid.size(); row++) {
//                JSONArray rowArray = (JSONArray) grid.get(row);
//                for (int col = 0; col < rowArray.size(); col++)
//                    this.grid[row][col] = (boolean) rowArray.get(col);
//            }
//
//            JSONObject fieldSize = (JSONObject) json.get("field_size");
//            fieldLength = ((Number) fieldSize.get("x")).doubleValue();
//            fieldWidth = ((Number) fieldSize.get("y")).doubleValue();
//        } catch (Exception ignored) {}
//    }
//
//    public boolean isInObstacle(Vector2D position) {
//        final int[] gridPos = getGridPos(position);
//        return grid[gridPos[0]][gridPos[1]];
//    }
//
//    /**
//     * @returns {row, col}
//     * */
//    public int[] getGridPos(Vector2D position) {
//        final double x = Math.min(fieldLength, Math.max(0, position.getX())),
//            y = Math.min(fieldWidth, Math.max(0, position.getY()));
//        return new int[]{(int) (y / cellSize), (int) (x / cellSize)};
//    }
//
//    public Vector2D getGridLowerLeftCornerPosition(int[] gridPos) {
//        return getGridLowerLeftCornerPosition(gridPos[0], gridPos[1]);
//    }
//
//    public Vector2D getGridLowerLeftCornerPosition(int row, int col) {
//        return new Vector2D(new double[] {col * cellSize, row * cellSize});
//    }
//
//    /**
//     * @return {boundedPosition, boundedVelocity}
//     */
//    public Vector2D[] applyCollisionDetection(Vector2D originalPosition, Vector2D
// originalVelocity) {
//        if (!isInObstacle(originalPosition))
//            return new Vector2D[] {originalPosition, originalVelocity};
//
//        // TODO: this is a very complicated program, write the explanations as fast as possible
//        if (originalVelocity.getX() > 0) { // horizontal velocity is to the right
//            if (isInObstacle(originalPosition.addBy(new Vector2D(new double[] {-cellSize, 0}))))
//                return originalVelocity.getY() > 0 ?
//                        applyUpwardsCollisionLimit(originalPosition, originalVelocity)
//                        :applyDownwardsCollisionLimit(originalPosition, originalVelocity);
//
//            if (originalVelocity.getY() > 0) // particle moving up-right
//                return linearInterpret(originalPosition, originalVelocity,
// getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX())
//                                >
// getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() ?
//                        applyRightwardsCollisionLimit(originalPosition, originalVelocity)
//                        :applyUpwardsCollisionLimit(originalPosition, originalVelocity);
//            if (originalVelocity.getY() < 0) // particle moving down-right
//                return linearInterpret(originalPosition, originalVelocity,
// getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX())
//                        < getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() +
// cellSize ?
//                        applyRightwardsCollisionLimit(originalPosition, originalVelocity)
//                        :applyDownwardsCollisionLimit(originalPosition, originalVelocity);
//            return applyRightwardsCollisionLimit(originalPosition, originalVelocity); // particle
// moving right
//        }
//        if (originalVelocity.getX() < 0) { // horizontal velocity is to the left
//            if (isInObstacle(originalPosition.addBy(new Vector2D(new double[] {cellSize, 0}))))
//                return originalVelocity.getY() > 0 ?
//                        applyUpwardsCollisionLimit(originalPosition, originalVelocity)
//                        :applyDownwardsCollisionLimit(originalPosition, originalVelocity);
//
//            if (originalVelocity.getY() > 0) // particle moving up-left
//                return linearInterpret(originalPosition, originalVelocity,
// getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX() + cellSize)
//                        > getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() ?
//                        applyLeftwardsCollisionLimit(originalPosition, originalVelocity)
//                        : applyUpwardsCollisionLimit(originalPosition, originalVelocity);
//            if (originalVelocity.getY() < 0) // particle moving down-left
//                return linearInterpret(originalPosition, originalVelocity,
// getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX() + cellSize)
//                        < getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() +
// cellSize ?
//                        applyLeftwardsCollisionLimit(originalPosition, originalVelocity)
//                        : applyDownwardsCollisionLimit(originalPosition, originalVelocity);
//            return applyLeftwardsCollisionLimit(originalPosition, originalVelocity); // particle
// moving left
//        }
//
//        // no horizontal velocity
//        if (originalPosition.getY() > 0) // particle moving up
//            return applyUpwardsCollisionLimit(originalPosition, originalVelocity);
//        if (originalPosition.getY() < 0) // particle moving down
//            return applyDownwardsCollisionLimit(originalPosition, originalVelocity);
//        return new Vector2D[] {originalPosition, originalVelocity}; // particle still
//    }
//
//    private double linearInterpret(Vector2D position, Vector2D velocity, double x) {
//        if (velocity.getX() == 0)
//            return Double.POSITIVE_INFINITY;
//        final double slope = velocity.getY() / velocity.getX();
//        return position.getY() + (x - position.getX()) * slope;
//    }
//
//    /**
//     * particle was moving up and hit the lower-edge of its current grid
//     * */
//    private Vector2D[] applyUpwardsCollisionLimit(Vector2D originalPosition, Vector2D
// originalVelocity) {
//        EasyDataFlow.putNumber("chassis physics simulation", "collision dir", 0);
//        if (Math.abs(originalVelocity.getY()) > maxImpactVelocity)
//            throw new RobotDamagedException();
//        return new Vector2D[] {
//                new Vector2D(new double[] {
//                        originalPosition.getX(),
//                        getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() -
// bounceBackDistance
//                }),
//                new Vector2D(new double[] {
//                        originalVelocity.getX(),
//                        originalVelocity.getY() * bounceCoefficient
//                })
//        };
//    }
//
//    /**
//     * particle was moving down and hit the upper-edge of its current grid
//     * */
//    private Vector2D[] applyDownwardsCollisionLimit(Vector2D originalPosition, Vector2D
// originalVelocity) {
//        EasyDataFlow.putNumber("chassis physics simulation", "collision dir", 1);
//        if (Math.abs(originalVelocity.getY()) > maxImpactVelocity)
//            throw new RobotDamagedException();
//        return new Vector2D[] {
//                new Vector2D(new double[] {
//                        originalPosition.getX(),
//                        getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getY() +
// cellSize + bounceBackDistance
//                }),
//                new Vector2D(new double[] {
//                        originalVelocity.getX(),
//                        originalVelocity.getY() * bounceCoefficient
//                })
//        };
//    }
//
//    /**
//     * particle was moving left and hit the righter-edge of its current grid
//     * */
//    private Vector2D[] applyLeftwardsCollisionLimit(Vector2D originalPosition, Vector2D
// originalVelocity) {
//        EasyDataFlow.putNumber("chassis physics simulation", "collision dir", 2);
//        if (Math.abs(originalVelocity.getX()) > maxImpactVelocity)
//            throw new RobotDamagedException();
//        return new Vector2D[] {
//                new Vector2D(new double[] {
//                        getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX() +
// cellSize + bounceBackDistance,
//                        originalPosition.getY()
//                }),
//                new Vector2D(new double[] {
//                        originalVelocity.getX() * bounceCoefficient,
//                        originalVelocity.getY()
//                })
//        };
//    }
//
//    /**
//     * particle was moving right and hit the lefter-edge of its current grid
//     * */
//    private Vector2D[] applyRightwardsCollisionLimit(Vector2D originalPosition, Vector2D
// originalVelocity) {
//        EasyDataFlow.putNumber("chassis physics simulation", "collision dir",  3);
//        if (Math.abs(originalVelocity.getX()) > maxImpactVelocity)
//            throw new RobotDamagedException();
//        return new Vector2D[] {
//                new Vector2D(new double[] {
//                        getGridLowerLeftCornerPosition(getGridPos(originalPosition)).getX() -
// bounceBackDistance,
//                        originalPosition.getY()
//                }),
//                new Vector2D(new double[] {
//                        originalVelocity.getX() * bounceCoefficient,
//                        originalVelocity.getY()
//                })
//        };
//    }
//
//    private static final class RobotDamagedException extends RuntimeException {
//        public RobotDamagedException() {
//            super("YOU CRUSHED THE ROBOT!!!!");
//        }
//    }
// }
