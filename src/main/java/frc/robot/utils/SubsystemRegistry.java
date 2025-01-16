package frc.robot.utils;

import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure;

public class SubsystemRegistry {
    private static SubsystemRegistry instance;
    private final Set<Subsystem> subsystems = new HashSet<>();

    private SubsystemRegistry() {
        // Add other subsystems to set
        subsystems.add(new Superstructure());
    }

    public static void initialize() {
        instance = new SubsystemRegistry();
    }

    @SuppressWarnings("unchecked")
    public static <T extends SubsystemBase> T getInstance(Class<T> subsystemClass) {
        return instance.subsystems.stream()
                .filter(subsystem -> subsystemClass.isInstance(subsystem))
                .map(subsystem -> (T) subsystem)
                .findFirst()
                .orElseThrow(() -> new RuntimeException("Subsystem not found: " + subsystemClass.getSimpleName()));
    }

    public static Set<StateManagedSubsystem> getAllStateSubsystems() {
        return instance.subsystems.stream()
                .filter(subsystem -> subsystem instanceof StateManagedSubsystem)
                .map(subsystem -> (StateManagedSubsystem) subsystem)
                .collect(Collectors.toSet());
    }

}
