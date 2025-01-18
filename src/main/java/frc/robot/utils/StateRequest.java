package frc.robot.utils;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;

public class StateRequest {
    private static Superstructure superstructure;
    private static final StateRequestHandler handler = new StateRequestHandler();
    private static final Map<Class<?>, Map<Enum<?>, Set<Enum<?>>>> stateExclusions = new HashMap<>();
    private static final Map<Class<?>, Enum<?>> currentStates = new HashMap<>();

    public static void init(Superstructure superstructure) {
        StateRequest.superstructure = superstructure;
    }

    @SafeVarargs
    public static <T extends Enum<T>> void addOneWayExclusion(T fromState, T... excludedStates) {
        Class<?> stateClass = fromState.getClass();
        stateExclusions.computeIfAbsent(stateClass, k -> new HashMap<>())
                .computeIfAbsent(fromState, k -> new HashSet<>());

        for (T excludedState : excludedStates) {
            stateExclusions.get(stateClass).get(fromState).add(excludedState);

        }
    }

    @SafeVarargs
    public static <T extends Enum<T>> void addTwoWayExclusion(T stateA, T... otherStates) {
        for (T stateB : otherStates) {
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
            if (stateObj instanceof Enum<?> enumValue) {
                @SuppressWarnings("unchecked")
                Class<? extends Enum<?>> stateClass = (Class<? extends Enum<?>>) enumValue.getClass();

                Enum<?> currentState = superstructure.states.get(stateClass);

                if (currentState != null) {
                    Enum<?> matchingKey = stateExclusions.get(stateClass).keySet().stream()
                            .filter(key -> key.toString().equals(currentState.toString()))
                            .findFirst()
                            .orElse(null);

                    if (matchingKey != null) {
                        Set<Enum<?>> exclusions = stateExclusions.get(stateClass).get(matchingKey);
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
                }
            }
        }

    }

    public static <T extends Enum<T>> IStateRequest create(T state) {
        IStateRequest request = (IStateRequest) Proxy.newProxyInstance(
                StateRequest.class.getClassLoader(),
                new Class<?>[] { IStateRequest.class },
                handler);
        handler.updateState(state);
        return request;
    }

    public static <T extends Enum<T>> T getCurrentState(Class<T> stateClass) {
        @SuppressWarnings("unchecked")
        T currentState = (T) currentStates.get(stateClass);
        return currentState;
    }
}
