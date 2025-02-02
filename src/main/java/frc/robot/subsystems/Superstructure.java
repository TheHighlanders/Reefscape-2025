package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class Superstructure extends SubsystemBase {
  public enum SuperstructureState {
    INIT,

    LOADING,

    DRIVING,

    L1,
    L2,
    L3,
    L4,
    CLIMBING,
  }

  public final Map<Class<? extends Enum<?>>, Enum<?>> states = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Field> stateFields = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Object> stateObjects = new HashMap<>();
  private Enum<?> lastState;
  public SuperstructureState currentState;
  private final Trigger stateChangeTrigger;

  public Superstructure(Map<String, Subsystem> subsystems) {
    currentState = SuperstructureState.INIT;
    // spotless:off
    /**
     *
     * For the haters:
     * We're using reflection to get the state fields of the subsystems.
     * Then storing these states in three tracking maps:
     * - states: Maps enum classes to their current enum values
     * - stateFields: Maps enum classes to their corresponding Field objects
     * - stateObjects: Maps enum classes to their parent subsystem instances
     * 
     * This allows us to easily access the current state of any subsystem.
     * And set the state of any subsystem.
     *
     */
    // spotless:on
    subsystems.values().stream()
        .filter(subsystem -> subsystem instanceof SubsystemBase)
        .forEach(
            subsystem -> {
              Field[] fields = subsystem.getClass().getDeclaredFields();
              for (Field field : fields) {
                if (field.getType().isEnum()) {
                  try {
                    field.setAccessible(true);
                    Object value = field.get(subsystem);
                    if (value instanceof Enum<?>) {
                      Enum<?> enumValue = (Enum<?>) value;
                      @SuppressWarnings("unchecked")
                      Class<? extends Enum<?>> enumClass =
                          (Class<? extends Enum<?>>) enumValue.getClass();
                      if (states.containsKey(enumClass)) {
                        System.err.println(
                            "ERROR: Duplicate enum type found: " + enumClass.getName());
                        continue;
                      }
                      states.put(enumClass, enumValue);
                      stateFields.put(enumClass, field);
                      stateObjects.put(enumClass, subsystem);
                    }
                  } catch (IllegalAccessException e) {
                    e.printStackTrace();
                  }
                }
              }
            });

    // Add the values to the fields manually because the superstructure cannot be
    // instantiated before it runs.
    try {
      Field field = Superstructure.class.getField("currentState");
      states.put(SuperstructureState.class, currentState);
      stateFields.put(SuperstructureState.class, field);
      stateObjects.put(SuperstructureState.class, this);
    } catch (NoSuchFieldException e) {
      e.printStackTrace();
    }

    stateChangeTrigger = getEnumChangeTrigger(SuperstructureState.class);
    stateChangeTrigger.onTrue(getStateChangeCommand());
  }

  @Override
  public void periodic() {}

  public Trigger getEnumChangeTrigger(Class<? extends Enum<?>> enumClass) {
    return new Trigger(
        () -> {
          Enum<?> currentState = states.get(enumClass);
          if (currentState != lastState) {
            lastState = currentState;
            return true;
          }
          return false;
        });
  }

  public Command getStateChangeCommand() {
    switch (currentState) {
      case INIT:
        return Commands.print("Superstructure: INIT");
      default:
        return Commands.print("Superstructure: DEFAULT");
    }
  }
}
