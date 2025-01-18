package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Superstructure extends SubsystemBase {
  public final Map<Class<? extends Enum<?>>, Enum<?>> states = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Field> stateFields = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Object> stateObjects = new HashMap<>();

  public Superstructure(List<Subsystem> subsystems) {
    subsystems.stream()
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
  }

  @Override
  public void periodic() {}
}
