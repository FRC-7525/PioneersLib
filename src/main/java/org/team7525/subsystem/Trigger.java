package org.team7525.subsystem;

import java.util.function.BooleanSupplier;

public class Trigger<StateType extends SubsystemStates> {

    BooleanSupplier supplier;
    StateType state;

    public Trigger(BooleanSupplier supplier, StateType state) {
        this.supplier = supplier;
        this.state = state;
    }

    public boolean isTriggered() {
        boolean triggered = supplier.getAsBoolean();
        return triggered;
    }

    public StateType getResultState() {
        return state;
    }
}
