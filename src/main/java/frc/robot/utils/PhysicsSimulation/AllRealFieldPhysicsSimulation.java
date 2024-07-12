//
//
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj.Timer;
// import frc.robot.Modules.PositionReader.RobotFieldPositionEstimator;
// import frc.robot.Utils.EasyDataFlow;
// import frc.robot.Utils.MathUtils.LookUpTable;
// import frc.robot.Utils.MathUtils.Rotation2D;
// import frc.robot.Utils.MathUtils.Vector2D;
// import frc.robot.Utils.PhysicsSimulation.FieldMaps.CrescendoDefault;
// import frc.robot.Utils.RobotConfigReader;
// import org.dyn4j.collision.CollisionBody;
// import org.dyn4j.collision.Fixture;
// import org.dyn4j.dynamics.Body;
// import org.dyn4j.dynamics.BodyFixture;
// import org.dyn4j.dynamics.Force;
// import org.dyn4j.dynamics.contact.Contact;
// import org.dyn4j.dynamics.contact.SolvedContact;
// import org.dyn4j.geometry.*;
// import org.dyn4j.world.ContactCollisionData;
// import org.dyn4j.world.PhysicsWorld;
// import org.dyn4j.world.World;
// import org.dyn4j.world.listener.ContactListener;
//
// import java.util.ArrayList;
// import java.util.List;
//
// public class AllRealFieldPhysicsSimulation {
//    public final World<Body> field;
//    private final FieldCollisionMap map;
//    private final List<HolomonicRobotPhysicsSimulation> robots;
//    private HolomonicRobotPhysicsSimulation mainRobot = null;
//    private final List<NoteOnField> notesOnField;
//    private final List<NotesOnFly> notesOnFly = new ArrayList<>();
//    public AllRealFieldPhysicsSimulation() {
//        this.robots = new ArrayList<>();
//        this.notesOnField = new ArrayList<>();
//        field = new World<>();
//        field.setGravity(PhysicsWorld.ZERO_GRAVITY);
//        map = new CrescendoDefault();
//        map.addObstaclesToField(field);
//    }
//
//    public void update(double dt) {
//        for (NoteOnField noteOnField:notesOnField)
//            noteOnField.update();
//        this.field.step(1, dt);
//
//        EasyDataFlow.putPositionArray("opponent robots", getOtherRobotsPositions2D(),
// getOtherRobotsRotations2D());
//        EasyDataFlow.putPosition3dArray("notePositions", getNotesPose3d());
//        removeArrivedNotesOnFly();
//        removeNotesByIntake();
//    }
//
//    public HolomonicRobotPhysicsSimulation setMainRobot(HolomonicRobotPhysicsSimulation robot) {
//        this.mainRobot = robot;
//        addRobot(mainRobot);
//        return mainRobot;
//    }
//
//    public HolomonicRobotPhysicsSimulation getMainRobot() {
//        return mainRobot;
//    }
//
//    public HolomonicRobotPhysicsSimulation addRobot(HolomonicRobotPhysicsSimulation robot) {
//        robots.add(robot);
//        field.addBody(robot);
//        return robot;
//    }
//
//    public Vector2D[] getOtherRobotsPositions2D() {
//        if (robots.size() <= 1)
//            return new Vector2D[] {};
//        Vector2D[] positions2D = new Vector2D[robots.size()-1];
//        int i = 0;
//        for (HolomonicRobotPhysicsSimulation robot:robots)
//            if (robot != this.mainRobot)
//                positions2D[i++] = robot.getFieldPosition();
//        return positions2D;
//    }
//
//    public Rotation2D[] getOtherRobotsRotations2D() {
//        if (robots.size() <= 1)
//            return new Rotation2D[] {};
//        Rotation2D[] rotations2D = new Rotation2D[robots.size()-1];
//        int i = 0;
//        for (HolomonicRobotPhysicsSimulation robot:robots)
//            if (robot != this.mainRobot)
//                rotations2D[i++] = robot.getFacing();
//        return rotations2D;
//    }
//
//    public NoteOnField addNoteToField(Vector2D startingPosition) {
//        final NoteOnField noteOnField = new NoteOnField(startingPosition);
//        notesOnField.add(noteOnField);
//        field.addBody(noteOnField);
//        return noteOnField;
//    }
//
//    public NoteOnField[] getNotesOnField() {
//        return notesOnField.toArray(new NoteOnField[0]);
//    }
//
//    public NoteOnField removeNoteOnField(NoteOnField noteOnField) {
//        field.removeBody(noteOnField);
//        notesOnField.remove(noteOnField);
//        return noteOnField;
//    }
//
//    public void removeAllNotesOnField() {
//        for (NoteOnField noteOnField:getNotesOnField())
//            removeNoteOnField(noteOnField);
//    }
//
//    public static final class RobotProfile {
//        public final double
//                robotMaxVelocity,
//                robotMaxAcceleration,
//                robotMass,
//                propellingForce,
//                frictionForce,
//                linearVelocityDamping,
//                maxAngularVelocity,
//                maxAngularAcceleration,
//                angularDamping,
//                angularFrictionAcceleration,
//                width,
//                height;
//
//        public RobotProfile(RobotConfigReader robotConfig) {
//            this(
//                    robotConfig.getConfig("chassis", "robotMaximumSpeed"),
//                    robotConfig.getConfig("chassis", "motorMaximumAcceleration"),
//                    robotConfig.getConfig("chassis", "floorFrictionAcceleration"),
//                    Math.toRadians(robotConfig.getConfig("chassis", "robotMaxAngularVelocity")),
//                    Math.toRadians(robotConfig.getConfig("chassis",
// "robotMaxAngularAcceleration")),
//                    robotConfig.getConfig("chassis", "timeChassisStopsRotating"),
//                    robotConfig.getConfig("chassis", "robotMass"),
//                    robotConfig.getConfig("chassis", "width"),
//                    robotConfig.getConfig("chassis", "height")
//            );
//        }
//        public RobotProfile(double robotMaxVelocity, double robotMaxAcceleration, double
// floorFrictionAcceleration, double maxAngularVelocity, double maxAngularAcceleration, double
// timeChassisStopsRotating, double robotMass, double width, double height) {
//            this.robotMaxVelocity = robotMaxVelocity;
//            this.robotMaxAcceleration = robotMaxAcceleration;
//            this.robotMass = robotMass;
//            this.propellingForce = robotMaxAcceleration * robotMass;
//            this.frictionForce = floorFrictionAcceleration * robotMass;
//            this.linearVelocityDamping = robotMaxAcceleration / robotMaxVelocity;
//            this.maxAngularVelocity = maxAngularVelocity;
//            this.maxAngularAcceleration = maxAngularAcceleration;
//            this.angularDamping = maxAngularAcceleration / maxAngularVelocity;
//            this.angularFrictionAcceleration = maxAngularVelocity / timeChassisStopsRotating;
//            this.width = width;
//            this.height = height;
//        }
//    }
//
//    public static class HolomonicRobotPhysicsSimulation extends Body {
//        public final RobotProfile profile;
//        public HolomonicRobotPhysicsSimulation(RobotProfile profile) {
//            this.profile = profile;
//
//            /* height and width is reversed */
//            super.addFixture(
//                    Geometry.createRectangle(profile.width, profile.height),
//                    profile.robotMass / (profile.height * profile.width),
//                    0.8,
//                    0.05
//            );
//
//            super.setMass(MassType.NORMAL);
//            super.setLinearDamping(profile.linearVelocityDamping);
//            super.setAngularDamping(profile.angularDamping);
//        }
//
//        public void setRobotPosition(Vector2D robotPositionOnField) {
//            super.transform.setTranslation(robotPositionOnField.getX(),
// robotPositionOnField.getY());
//        }
//
//        public void setRobotRotation(Rotation2D robotFacing) {
//            super.transform.setRotation(robotFacing.getRadian());
//        }
//
//        public void resetMotion() {
//            setMotion(new Vector2D(), 0);
//        }
//
//        public void setMotion(Vector2D linearVelocity, double angularVelocity) {
//            super.setLinearVelocity(Vector2D.toVector2(linearVelocity));
//            super.setAngularVelocity(angularVelocity);
//        }
//
//        public void simulateChassisTranslationalBehavior(Vector2D desiredMotionToRobot) {
//
// simulateChassisTranslationalBehaviorFieldOriented(desiredMotionToRobot.multiplyBy(getFacing()));
//        }
//
//        public void simulateChassisTranslationalBehaviorFieldOriented(Vector2D
// desiredMotionToField) {
//            super.setAtRest(false);
//            if (desiredMotionToField.getMagnitude() > 0.03)
//                super.applyForce(new Force(Vector2D.toVector2(
//                        desiredMotionToField.multiplyBy(this.profile.propellingForce))));
//            else {
//                if (Vector2D.fromVector2(super.getLinearVelocity()).getMagnitude() > 0.03 *
// this.profile.robotMaxVelocity)
//                    super.applyForce(new Force(Vector2D.toVector2(
//                            new
// Vector2D(Vector2D.fromVector2(super.getLinearVelocity()).getHeading(),
// -this.profile.frictionForce)
//                    )));
//                else
//                    super.setLinearVelocity(0, 0);
//            }
//        }
//
//        public void simulateChassisRotationalBehavior(double rotationPower) {
//            EasyDataFlow.putNumber("chassis physics simulation", "desired rotational motion",
// rotationPower);
//            if (Math.abs(rotationPower) > 0.05)
//                super.applyTorque(rotationPower * this.profile.maxAngularAcceleration *
// super.getMass().getInertia());
//            else {
//                if (Math.abs(super.getAngularVelocity()) < this.profile.maxAngularVelocity * 0.05)
//                    super.setAngularVelocity(0);
//                else
//                    super.applyTorque(Math.copySign(this.profile.angularFrictionAcceleration *
// super.getMass().getInertia(), -super.getAngularVelocity()));
//            }
//        }
//
//        public Vector2D getFieldPosition() {
//            return Vector2D.fromVector2(super.transform.getTranslation());
//        }
//
//        public Vector2D getFieldVelocity() {
//            return Vector2D.fromVector2(super.linearVelocity);
//        }
//
//        public Rotation2D getFacing() {
//            return Rotation2D.fromTransform(super.transform);
//        }
//    }
//
//    /* remove the notes after detection listening to avoid ConcurrentModificationException */
//    private void removeNotesByIntake() {
//        for (NoteOnField noteToRemove:IntakeSimulation.notesToRemove)
//            removeNoteOnField(noteToRemove);
//        IntakeSimulation.notesToRemove.clear();
//    }
//    public static class IntakeSimulation extends BodyFixture {
//        private static final List<NoteOnField> notesToRemove = new ArrayList<>();
//        private boolean intakeEnabled;
//        public IntakeSimulation(Convex shape) {
//            super(shape);
//            this.intakeEnabled = false;
//        }
//
//        public void setIntakeEnabled(boolean enabled) {
//            this.intakeEnabled = enabled;
//        }
//
//        public ContactListener<Body> getGamePieceOnFieldContactListener() {
//            return new ContactListener<>() {
//                @Override
//                public void begin(ContactCollisionData collision, Contact contact) {
//                    if (!intakeEnabled)
//                        return;
//                    CollisionBody<?> collisionBody1 = collision.getBody1();
//                    CollisionBody<?> collisionBody2 = collision.getBody2();
//                    Fixture fixture1 = collision.getFixture1();
//                    Fixture fixture2 = collision.getFixture2();
//
//                    if (collisionBody1 instanceof NoteOnField && fixture2 ==
// IntakeSimulation.this)
//                        notesToRemove.add((NoteOnField) collisionBody1);
//                    else if (collisionBody2 instanceof NoteOnField && fixture1 ==
// IntakeSimulation.this)
//                        notesToRemove.add((NoteOnField) collisionBody2);
//                }
//
//                /* functions not used */
//                @Override
//                public void persist(ContactCollisionData collision, Contact oldContact, Contact
// newContact) {
//                }
//
//                @Override
//                public void end(ContactCollisionData collision, Contact contact) {
//                }
//
//                @Override
//                public void destroyed(ContactCollisionData collision, Contact contact) {
//                }
//
//                @Override
//                public void collision(ContactCollisionData collision) {
//                }
//
//                @Override
//                public void preSolve(ContactCollisionData collision, Contact contact) {
//                }
//
//                @Override
//                public void postSolve(ContactCollisionData collision, SolvedContact contact) {
//                }
//            };
//        }
//    }
//
//    public static class NoteOnField extends Body {
//        public static final double
//                noteRadius = 0.1778,
//                noteMass = 0.235,
//                frictionAcceleration = -4,
//                noteHeight = 0.05;
//        public NoteOnField(Vector2D startingPosition) {
//            /* height and width is reversed */
//            BodyFixture bodyFixture = super.addFixture(Geometry.createCircle(noteRadius));
//            bodyFixture.setFriction(0.8);
//            bodyFixture.setRestitution(0.1);
//            super.setMass(new Mass(new Vector2(), noteMass, Double.POSITIVE_INFINITY));
//            super.translate(Vector2D.toVector2(startingPosition));
//        }
//
//        public void update() {
//            super.setAtRest(false);
//            if (getFieldVelocity().getMagnitude() > 0.3)
//                super.applyForce(Vector2D.toVector2(new Vector2D(getFieldVelocity().getHeading(),
// frictionAcceleration * super.getMass().getMass())));
//            else
//                super.setLinearVelocity(0, 0);
//        }
//
//        public Vector2D getFieldPosition() {
//            return Vector2D.fromVector2(super.transform.getTranslation());
//        }
//
//        public Vector2D getFieldVelocity() {
//            return Vector2D.fromVector2(super.linearVelocity);
//        }
//    }
//
//    public static class NotesOnFly {
//        public static final Vector2D blueSpeakerPosition = new Vector2D(new double[] {0, 5.55});
//        public static final double speakerHeight = 2.2;
//        private final Vector2D launcherPosition, speakerPosition;
//        private final double launcherHeight, launchTime, timeArrival;
//        private final Rotation3d rotation3d;
//        public NotesOnFly(Vector2D launcherPosition, double launcherHeight, double launchSpeed) {
//            this.launcherPosition = launcherPosition;
//            this.launcherHeight = launcherHeight;
//            this.speakerPosition =
// RobotFieldPositionEstimator.toActualPositionOnField(blueSpeakerPosition);
//            this.launchTime = Timer.getFPGATimestamp();
//            final double timeNeeded =
//                    Math.sqrt(
//                            Math.pow(Vector2D.displacementToTarget(launcherPosition,
// speakerPosition).getMagnitude(), 2)
//                                    + Math.pow(speakerHeight - launcherHeight, 2)
//                    ) / launchSpeed;
//            this.timeArrival = launchTime + timeNeeded;
//
//            final Vector2D displacement = Vector2D.displacementToTarget(launcherPosition,
// speakerPosition);
//            final double yaw = displacement.getHeading(),
//                    pitch = -Math.atan2(speakerHeight - launcherHeight,
// displacement.getMagnitude());
//            this.rotation3d = new Rotation3d(0, pitch, yaw);
//        }
//
//        public Pose3d getPose3d() {
//            final double
//                    x = LookUpTable.linearInterpretationWithBounding(launchTime,
// launcherPosition.getX(), launchTime+0.5, speakerPosition.getX(), Timer.getFPGATimestamp()),
//                    y = LookUpTable.linearInterpretationWithBounding(launchTime,
// launcherPosition.getY(), launchTime+0.5, speakerPosition.getY(), Timer.getFPGATimestamp()),
//                    z = LookUpTable.linearInterpretationWithBounding(launchTime, launcherHeight,
// launchTime+0.5, speakerHeight, Timer.getFPGATimestamp());
//            return new Pose3d(new Translation3d(x, y, z), rotation3d);
//        }
//
//        public boolean arrived() {
//            return Timer.getFPGATimestamp() >= timeArrival;
//        }
//    }
//
//    private static final double launcherHeight = 0.3, launchSpeed = 10;
//    private static final Vector2D shooterPositionOnRobot = new Vector2D(new double[] {0, 0});
//    public void launchNote(Vector2D robotFieldPosition) {
//        final double robotRotationRadian = Vector2D.displacementToTarget(robotFieldPosition,
// RobotFieldPositionEstimator.toActualPositionOnField(NotesOnFly.blueSpeakerPosition)).getHeading();
//        final Vector2D launcherPositionField =
//                robotFieldPosition.addBy(shooterPositionOnRobot.multiplyBy(new
// Rotation2D(robotRotationRadian - Math.toRadians(90))));
//        this.notesOnFly.add(new NotesOnFly(launcherPositionField, launcherHeight, launchSpeed));
//    }
//
//    public Pose3d[] getNotesPose3d() {
//        final Pose3d[] pose3ds = new Pose3d[notesOnField.size() + notesOnFly.size()];
//        for (int i = 0; i < notesOnField.size(); i++) {
//            final Vector2D notePosition2D = notesOnField.get(i).getFieldPosition();
//            pose3ds[i] = new Pose3d(
//                    new Translation3d(notePosition2D.getX(), notePosition2D.getY(),
// NoteOnField.noteHeight / 2),
//                    new Rotation3d()
//            );
//        }
//        for (int i = 0; i < notesOnFly.size(); i++)
//            pose3ds[i + notesOnField.size()] = notesOnFly.get(i).getPose3d();
//        return pose3ds;
//    }
//
//    public NotesOnFly removeArrivedNotesOnFly() {
//        for (int i =0; i < notesOnFly.size(); i++)
//            if (notesOnFly.get(i).arrived())
//                return notesOnFly.remove(i);
//        return null;
//    }
//
//
//    public static abstract class FieldCollisionMap {
//        private final List<Body> obstacles = new ArrayList<>();
//
//        protected void addLinearObstacle(Vector2D startingPoint, Vector2D endingPoint) {
//            addRectangularObstacle(
//                    startingPoint.addBy(Vector2D.displacementToTarget(startingPoint,
// endingPoint).multiplyBy(0.5)),
//                    Vector2D.displacementToTarget(startingPoint, endingPoint).getMagnitude(),
//                    0.01,
//                    new Rotation2D(Vector2D.displacementToTarget(startingPoint,
// endingPoint).getHeading())
//            );
//        }
//
//        protected void addRectangularObstacle(Vector2D centerPosition, double width, double
// height, Rotation2D rotation) {
//            final Body obstacle = new Body();
//            obstacle.setMass(MassType.INFINITE);
//            final BodyFixture fixture = obstacle.addFixture(Geometry.createRectangle(width,
// height));
//            fixture.setFriction(0.8);
//            fixture.setRestitution(0.6);
//            obstacle.getTransform().setTranslation(Vector2D.toVector2(centerPosition));
//            obstacle.getTransform().setRotation(rotation.getRadian());
//            addCustomObstacle(obstacle);
//        }
//
//        protected void addCustomObstacle(Body customObstacle) {
//            obstacles.add(customObstacle);
//        }
//
//        public void addObstaclesToField(World<Body> field) {
//            for (Body obstacle:obstacles)
//                field.addBody(obstacle);
//        }
//    }
// }
