package org.firstinspires.ftc.teamcode.Core.Algorithms.ActionSystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SequentialAction extends AbstractAction {
    private final List<AbstractAction> actions = new ArrayList<>();
    private int currentIndex = 0;
    private boolean running = false;

    public SequentialAction(AbstractAction... actions) {
        this.actions.addAll(Arrays.asList(actions));
    }

    @Override
    public void start() {
        if (actions.isEmpty()) return;
        currentIndex = 0;
        running = true;
        actions.get(0).start();
    }

    @Override
    public void exec() {
        if (!running || actions.isEmpty()) return;

        AbstractAction currentAction = actions.get(currentIndex);

        currentAction.exec();

        if (!currentAction.isRunning()) {
            currentIndex++;

            if (currentIndex >= actions.size()) {
                end();
            } else {
                actions.get(currentIndex).start();
            }
        }
    }

    @Override
    public void end() {
        running = false;
        currentIndex = 0;
        for (AbstractAction a : actions) {
            if (a.isRunning()) a.end();
        }
    }

    @Override
    public boolean isRunning() {
        return running;
    }
}