package frc.robot.utils;

import frc.robot.subsystems.Superstructure;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class StateRequest {
  private static Superstructure superstructure;
  private static final StateRequestHandler handler = new StateRequestHandler();
  private static final Map<Class<?>, Map<Enum<?>, Set<Enum<?>>>> stateExclusions = new HashMap<>();

  public static void init(Superstructure superstructure) {
    StateRequest.superstructure = superstructure;
  }

  public static void addOneWayExclusion(Enum<?> fromState, Enum<?>... excludedStates) {
    Class<?> stateClass = fromState.getClass();
    stateExclusions
        .computeIfAbsent(stateClass, k -> new HashMap<>())
        .computeIfAbsent(fromState, k -> new HashSet<>());

    for (Enum<?> excludedState : excludedStates) {
      stateExclusions.get(stateClass).get(fromState).add(excludedState);
    }
  }

  public static void addTwoWayExclusion(Enum<?> stateA, Enum<?>... otherStates) {
    for (Enum<?> stateB : otherStates) {
      addOneWayExclusion(stateA, stateB);
      addOneWayExclusion(stateB, stateA);
    }
  }

  private static class StateRequestHandler implements InvocationHandler {
    @Override
    public Object invoke(Object proxy, Method method, Object[] args) {
      updateState(args[0]);
      return proxy;
    }

    private <T extends Enum<T>> void updateState(Object stateObj) {
      if (superstructure == null) return;

      if (stateObj instanceof Enum<?> enumValue) {
        @SuppressWarnings("unchecked")
        Class<? extends Enum<?>> stateClass = (Class<? extends Enum<?>>) enumValue.getClass();

        for (Map.Entry<Class<?>, Map<Enum<?>, Set<Enum<?>>>> entry : stateExclusions.entrySet()) {
          Class<?> exclusionClass = entry.getKey();
          Enum<?> currentState = superstructure.states.get(exclusionClass);

          if (currentState != null) {
            Set<Enum<?>> exclusions = entry.getValue().get(currentState);
            if (exclusions != null && exclusions.contains(enumValue)) {
              return;
            }
          }
        }

        if (superstructure.states.containsKey(stateClass)) {
          superstructure.states.put(stateClass, enumValue);
          Field field = superstructure.stateFields.get(stateClass);
          Object instance = superstructure.stateObjects.get(stateClass);
          try {
            field.set(instance, enumValue);
          } catch (IllegalAccessException e) {
            e.printStackTrace();
          }
        } else {
          System.out.println("State " + stateClass.getSimpleName() + " not found in Superstructure");
          System.out.println(superstructure.states.keySet());
        }
      }
    }
  }

  /**
   * Creates a new {@link IStateRequest} instance that represents a request to update the state of a subsystem.
   *
   * @param <T> The enum type of the state being requested.
   * @param state The new state to be set.
   * @return A new {@link IStateRequest} instance that can be used to update the state.
   */
  public static <T extends Enum<T>> IStateRequest create(T state) {
    IStateRequest request =
        (IStateRequest)
            Proxy.newProxyInstance(
                StateRequest.class.getClassLoader(), new Class<?>[] {IStateRequest.class}, handler);
    handler.updateState(state);
    return request;
  }

  /**
   * Gets the current state of a subsystem's enum.
   *
   * @param <T> The enum type parameter
   * @param stateClass The class of the enum to get the state of
   * @return The current state of the specified enum type or null if the state doesn't exist
   * @see frc.robot.subsystems.Superstructure
   */
  public static <T extends Enum<T>> T getCurrentState(Class<T> stateClass) {
    if (superstructure == null) return null;
    @SuppressWarnings("unchecked")
    T currentState = (T) superstructure.states.get(stateClass);
    return currentState;
  }

  public static <T extends Enum<T>> BooleanSupplier createStateChangeSupplier(Class<T> enumClass) {
    return new BooleanSupplier() {
      T lastState = getCurrentState(enumClass);

      @Override
      public boolean getAsBoolean() {
        T currentState = getCurrentState(enumClass);
        if (currentState != lastState) {
          lastState = currentState;
          return true;
        }
        return false;
      }
    };
  }
}
