// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {
  public final Map<Class<? extends Enum<?>>, Enum<?>> states = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Field> stateFields = new HashMap<>();
  public final Map<Class<? extends Enum<?>>, Object> stateObjects = new HashMap<>();

  @SuppressWarnings("unchecked")
  public Superstructure(Set<Subsystem> subsystems) {
    subsystems.stream()
        .filter(subsystem -> subsystem instanceof SubsystemBase)
        .forEach(subsystem -> {
          Field[] fields = subsystem.getClass().getDeclaredFields();
          for (Field field : fields) {
            if (field.getType().isEnum()) {
              try {
                field.setAccessible(true);
                Enum<?> enumValue = (Enum<?>) field.get(subsystem);
                if (enumValue != null) {
                  states.put((Class<? extends Enum<?>>) field.getType(), enumValue);
                  stateFields.put((Class<? extends Enum<?>>) field.getType(), field);
                  stateObjects.put((Class<? extends Enum<?>>) field.getType(), subsystem);
                }
              } catch (IllegalAccessException e) {
                e.printStackTrace();
              }
            }
          }
        });
  }

  @Override
  public void periodic() {
  }
}
