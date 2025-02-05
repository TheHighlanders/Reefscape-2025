package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SuperstructureTest.TestSubsystem.FirstEnumState;
import frc.robot.utils.StateRequest;
import java.util.HashMap;
import java.util.Map;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class SuperstructureTest {
  private Superstructure superstructure;
  private TestSubsystem testSubsystem;

  public static class TestSubsystem extends SubsystemBase {
    public enum FirstEnumState {
      ONE,
      TWO,
      THREE,
      FOUR,
      FIVE
    }

    public enum SecondEnumState {
      A,
      B,
      C
    }

    public FirstEnumState state = FirstEnumState.ONE;
    public SecondEnumState testState = SecondEnumState.A;
  }

  @BeforeEach
  void setUp() {
    Map<String, Subsystem> subsystems = new HashMap<>();
    testSubsystem = new TestSubsystem();
    subsystems.put("test", testSubsystem);
    superstructure = new Superstructure(subsystems);
    StateRequest.init(superstructure);
  }

  @Test
  void testForwardExclusion() {
    StateRequest.addOneWayExclusion(
        TestSubsystem.FirstEnumState.ONE, TestSubsystem.FirstEnumState.THREE);
    StateRequest.create(TestSubsystem.FirstEnumState.ONE);
    StateRequest.create(TestSubsystem.FirstEnumState.THREE);

    assertEquals(
        TestSubsystem.FirstEnumState.ONE,
        StateRequest.getCurrentState(TestSubsystem.FirstEnumState.class));
  }

  @Test
  void testReverseDirection() {
    StateRequest.addOneWayExclusion(
        TestSubsystem.FirstEnumState.ONE, TestSubsystem.FirstEnumState.THREE);
    StateRequest.create(TestSubsystem.FirstEnumState.THREE);
    StateRequest.create(TestSubsystem.FirstEnumState.ONE);

    assertEquals(
        TestSubsystem.FirstEnumState.ONE,
        StateRequest.getCurrentState(TestSubsystem.FirstEnumState.class));
  }

  @Test
  void testStateChange() {
    var stateChanged = StateRequest.createStateChangeSupplier(TestSubsystem.FirstEnumState.class);
    StateRequest.create(TestSubsystem.FirstEnumState.TWO);

    assertTrue(stateChanged.getAsBoolean());
  }

  @Test
  void testNoStateChange() {
    var stateChanged = StateRequest.createStateChangeSupplier(TestSubsystem.FirstEnumState.class);
    StateRequest.create(TestSubsystem.FirstEnumState.ONE);

    assertFalse(stateChanged.getAsBoolean());
  }

  @Test
  void testCrossEnumExclusion() {
    StateRequest.addOneWayExclusion(
        TestSubsystem.FirstEnumState.ONE, TestSubsystem.SecondEnumState.B);
    StateRequest.create(TestSubsystem.FirstEnumState.ONE);
    StateRequest.create(TestSubsystem.SecondEnumState.B);

    assertEquals(
        TestSubsystem.SecondEnumState.A,
        StateRequest.getCurrentState(TestSubsystem.SecondEnumState.class));
  }

  @Test
  void testStateRequest() {

    StateRequest.create(FirstEnumState.ONE);
    assertEquals(
        FirstEnumState.ONE, StateRequest.getCurrentState(TestSubsystem.FirstEnumState.class));
  }
}
