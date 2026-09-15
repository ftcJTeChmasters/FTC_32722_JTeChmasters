package org.firstinspires.ftc.teamcode.util;

/**
 * A non-blocking wait/timer you can poll from your OpMode loop.
 * Usage:
 *   NonBlockingWait wait = new NonBlockingWait();
 *   wait.start(1000); // ms
 *   ...
 *   if (wait.isDone()) { ... }
 */
public class NonBlockingWait {
    private long startTimeNs = 0;
    private long durationNs = 0;
    private boolean running = false;

    /** Start (or restart) the wait for the given duration in milliseconds. */
    public void start(double durationMs) {
        this.startTimeNs = System.nanoTime();
        this.durationNs = (long) (durationMs * 1_000_000L);
        this.running = true;
    }

    /** Returns true once the duration has elapsed since start(). */
    public boolean isDone() {
        if (!running) return false;
        boolean done = (System.nanoTime() - startTimeNs) >= durationNs;
        if (done) running = false; // auto-clear so isDone() doesn't fire repeatedly
        return done;
    }

    /** True while the timer is actively counting down. */
    public boolean isRunning() {
        return running;
    }

    /** Elapsed time in milliseconds since start() was called. */
    public double elapsedMs() {
        return (System.nanoTime() - startTimeNs) / 1_000_000.0;
    }

    /** Remaining time in milliseconds (0 if done or not running). */
    public double remainingMs() {
        if (!running) return 0;
        double remaining = (durationNs - (System.nanoTime() - startTimeNs)) / 1_000_000.0;
        return Math.max(0, remaining);
    }

    /** Cancel the wait early. */
    public void cancel() {
        running = false;
    }
}