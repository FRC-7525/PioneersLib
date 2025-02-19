package org.team7525.subsystem;

import java.util.function.BooleanSupplier;

public class RunnableTrigger {

    BooleanSupplier supplier;
    Runnable runnable;

    public RunnableTrigger(BooleanSupplier supplier, Runnable runnable) {
        this.supplier = supplier;
        this.runnable = runnable;
    }

    public boolean isTriggered() {
        return supplier.getAsBoolean();
    }

    public void run() {
        runnable.run();
    }
}
