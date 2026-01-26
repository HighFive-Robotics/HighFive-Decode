package org.firstinspires.ftc.teamcode.Core.Algorithms.ActionSystem;

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class CoreAction extends AbstractAction {

    private boolean running = false;
    private String name = "Unnamed Action";

    private final ElapsedTime masterTimer = new ElapsedTime();

    private double timeoutSeconds = -1;

    private final Map<String, ElapsedTime> auxTimers = new HashMap<>();

    private Consumer<CoreAction> startAction = (a) -> {};
    private Consumer<CoreAction> endAction = (a) -> {};

    private Predicate<CoreAction> loopLogic = (a) -> true;

    public static CoreAction builder() {
        return new CoreAction();
    }

    public CoreAction withName(String name) {
        this.name = name;
        return this;
    }

    public CoreAction withTimeout(double seconds) {
        this.timeoutSeconds = seconds;
        return this;
    }

    public CoreAction onStart(Consumer<CoreAction> action) {
        this.startAction = action;
        return this;
    }

    public CoreAction onStart(Runnable action) {
        this.startAction = (a) -> action.run();
        return this;
    }

    public CoreAction loop(Predicate<CoreAction> logic) {
        this.loopLogic = logic;
        return this;
    }

    public CoreAction loop(java.util.function.BooleanSupplier logic) {
        this.loopLogic = (a) -> logic.getAsBoolean();
        return this;
    }

    public CoreAction onEnd(Consumer<CoreAction> action) {
        this.endAction = action;
        return this;
    }
    public CoreAction onEnd(Runnable action) {
        this.endAction = (a) -> action.run();
        return this;
    }

    public double time() {
        return masterTimer.seconds();
    }

    public ElapsedTime timer(String name) {
        if (!auxTimers.containsKey(name)) {
            ElapsedTime newTimer = new ElapsedTime();
            auxTimers.put(name, newTimer);

            if(running) newTimer.reset();
        }
        return auxTimers.get(name);
    }

    @Override
    public void start() {
        running = true;
        masterTimer.reset();

        for (ElapsedTime t : auxTimers.values()) {
            t.reset();
        }
        startAction.accept(this);
    }

    @Override
    public void exec() {
        if (running) {

            if (timeoutSeconds > 0 && masterTimer.seconds() >= timeoutSeconds) {
                end();
                return;
            }

            boolean isFinished = loopLogic.test(this);

            if (isFinished) {
                end();
            }
        }
    }

    @Override
    public void end() {
        running = false;
        endAction.accept(this);
    }

    @Override
    public boolean isRunning() {
        return running;
    }

    @Override
    public String toString() {
        return "Action[" + name + "]";
    }
}