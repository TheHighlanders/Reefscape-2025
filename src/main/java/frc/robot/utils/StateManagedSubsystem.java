package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface StateManagedSubsystem extends Subsystem {
    Class<? extends Enum<?>> getStateType();

    StateHandler<?> getStateHandler();
}
