package org.team7525.subsystem;

import java.util.function.BooleanSupplier;

public class Trigger<StateType extends SubsystemStates> {

    Runnable onEnd;
    BooleanSupplier supplier;
    StateType state;

    public Trigger(BooleanSupplier supplier, StateType state, Runnable onEnd) {
        this.supplier = supplier;
        this.state = state;
        this.onEnd = onEnd;
    }

    public Trigger(BooleanSupplier supplier, StateType state) {
        this(supplier, state, () -> {});
    }

    public boolean isTriggered() {
        boolean triggered = supplier.getAsBoolean();
        return triggered;
    }

    public StateType getResultState() {
        return state;
    }
}
