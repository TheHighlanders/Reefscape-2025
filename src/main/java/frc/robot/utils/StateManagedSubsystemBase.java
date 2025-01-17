package frc.robot.utils;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class StateManagedSubsystemBase<T extends Enum<T>> extends SubsystemBase
        implements StateManagedSubsystem<T> {

    public StateManagedSubsystemBase() {
        super();
    }

    @Override
    @SuppressWarnings("unchecked")
    public Class<T> getStateType() {
        var enums = Arrays.stream(getClass().getDeclaredClasses())
                .filter(Class::isEnum)
                .toList();

        if (enums.size() == 1) {
            return (Class<T>) enums.get(0);
        }

        return (Class<T>) enums.stream()
                .filter(e -> e.getSimpleName().toLowerCase().contains("state"))
                .findFirst()
                .orElseThrow(() -> new RuntimeException("No state enum found in " + getClass().getSimpleName()));
    }

    @Override
    @SuppressWarnings("unchecked")
    public StateHandler<T> getStateHandler() {
        return (StateHandler<T>) Arrays.stream(getClass().getDeclaredFields())
                .filter(field -> field.getType().equals(StateHandler.class))
                .findFirst()
                .map(field -> {
                    field.setAccessible(true);
                    try {
                        return field.get(this);
                    } catch (IllegalAccessException e) {
                        throw new RuntimeException("Could not access StateHandler in " + getClass().getSimpleName());
                    }
                })
                .orElseThrow(() -> new RuntimeException("No StateHandler found in " + getClass().getSimpleName()));
    }
}
