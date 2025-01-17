package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface StateManagedSubsystem<T extends Enum<T>> extends Subsystem {
    Class<T> getStateType();
    StateHandler<T> getStateHandler();
}
