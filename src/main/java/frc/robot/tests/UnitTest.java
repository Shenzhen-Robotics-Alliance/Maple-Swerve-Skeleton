package frc.robot.tests;

public interface UnitTest {
    void testStart();
    void testPeriodic();
    default void testEnd() {}
}
