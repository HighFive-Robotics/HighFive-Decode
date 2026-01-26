package org.firstinspires.ftc.teamcode.Core.Algorithms.ActionSystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction extends AbstractAction {
    private final List<AbstractAction> actions = new ArrayList<>();
    private boolean running = false;

    public ParallelAction(AbstractAction... actions) {
        this.actions.addAll(Arrays.asList(actions));
    }

    @Override
    public void start() {
        if (actions.isEmpty()) return;
        running = true;
        for (AbstractAction action : actions) {
            action.start();
        }
    }

    @Override
    public void exec() {
        if (!running) return;

        boolean anyChildRunning = false;

        for (AbstractAction action : actions) {
            if (action.isRunning()) {
                action.exec();
                anyChildRunning = true;
            }
        }
        if (!anyChildRunning) {
            end();
        }
    }

    @Override
    public void end() {
        running = false;
        for (AbstractAction action : actions) {
            if (action.isRunning()) {
                action.end();
            }
        }
    }

    @Override
    public boolean isRunning() {
        return running;
    }
}