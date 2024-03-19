package frc.robot.utils;

import java.util.LinkedList;
import java.util.Optional;
import java.util.Queue;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Predicate;

public class TaskRunner<T> {
    private Queue<Task<T>> tasks;
    private Optional<Consumer<T>> defaultTask;

    public TaskRunner() {
        tasks = new LinkedList<Task<T>>();
        defaultTask = Optional.empty();
    }

    public TaskRunner<T> withDefault(Consumer<T> consumer) {
        defaultTask = Optional.of(consumer);
        return this;
    }

    public TaskRunner<T> then(Task<T> task) {
        tasks.add(task);
        return this;
    }

    public void runOnce(T parameter) {
        if (tasks.isEmpty()) {
            defaultTask.ifPresent((t) -> t.accept(parameter));
            return;
        }
        
        var currentTask = tasks.element();
        currentTask.run(parameter);
        if (currentTask.isFinished(parameter)) {
            tasks.remove();
        }
    }

    public void clear() {
        tasks.clear();
    }

    public boolean isBusy() {
        return !tasks.isEmpty();
    }

    public static class Task<T> {
        private Consumer<T> task;
        private TaskType type;
        private BooleanSupplier completedSupp;
        private Predicate<T> completedPred;
        private int duration;
        private int timer;

        public Task(Consumer<T> task) {
            this.task = task;
            this.type = TaskType.Infinite;
        }

        public Task(Consumer<T> task, int duration) {
            this.task = task;
            this.type = TaskType.Duration;
            this.timer = 0;
            this.duration = duration;
        }

        public Task(Consumer<T> task, BooleanSupplier toWaitFor) {
            this.task = task;
            this.type = TaskType.WaitForCallback;
            this.completedSupp = toWaitFor;
        }

        public Task(Consumer<T> task, Predicate<T> toWaitFor) {
            this.task = task;
            this.type = TaskType.WaitForPredicate;
            this.completedPred = toWaitFor;
        }
        
        private void run(T input) {
            task.accept(input);
        }

        private boolean isFinished(T process) {
            switch (this.type) {
                case Infinite: return false;
                case Duration:
                {
                    if (duration == timer) {
                        return true;
                    }
                    ++timer;
                    return false;
                }
                case WaitForCallback: return completedSupp.getAsBoolean();
                case WaitForPredicate: return completedPred.test(process);
                default: return true;
            }
        }

        public static enum TaskType {
            Infinite,
            Duration,
            WaitForCallback,
            WaitForPredicate,
        }
    }
}
