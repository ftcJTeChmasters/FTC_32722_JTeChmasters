package org.firstinspires.ftc.teamcode.util;

/**
 * A non-blocking wait/timer you can poll from your OpMode loop.
 * <p>
 * Unlike {@code sleep()}, this doesn't block the thread — you call
 * {@link #start(double)} once, then poll {@link #isDone()} each loop.
 * <p>
 * Example usage:
 * <pre>{@code
 * NonBlockingWait wait = new NonBlockingWait();
 * wait.start(1000); // 1 second
 *
 * while (opModeIsActive()) {
 *     if (wait.isDone()) {
 *         // do something after 1 second
 *     }
 * }
 * }</pre>
 */
public class NonBlockingWait {
    private long startTimeNanoseconds = 0;
    private long durationNanoseconds = 0;
    private boolean running = false;

    /**
     * Starts (or restarts) the wait for the given duration in milliseconds.
     *
     * @param durationMilliseconds duration to wait in milliseconds
     */
    public void start(double durationMilliseconds) {
        this.startTimeNanoseconds = System.nanoTime();
        this.durationNanoseconds = (long) (durationMilliseconds * 1_000_000L);
        this.running = true;
    }

    /**
     * Returns {@code true} once the duration has elapsed since {@link #start(double)}.
     * <p>
     * After returning {@code true} once, automatically clears the running state
     * so subsequent calls return {@code false} until {@link #start(double)} is
     * called again.
     *
     * @return {@code true} if the wait is complete, {@code false} otherwise
     */
    public boolean isDone() {
        if (!running) return false;
        boolean done = (System.nanoTime() - startTimeNanoseconds) >= durationNanoseconds;
        if (done) running = false; // auto-clear so isDone() doesn't fire repeatedly
        return done;
    }

    /**
     * Returns {@code true} while the timer is actively counting down.
     *
     * @return {@code true} if the timer is running, {@code false} otherwise
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Returns the elapsed time in milliseconds since {@link #start(double)} was called.
     *
     * @return elapsed time in milliseconds, or 0 if not running
     */
    public double elapsedMilliseconds() {
        return (System.nanoTime() - startTimeNanoseconds) / 1_000_000.0;
    }

    /**
     * Returns the remaining time in milliseconds.
     *
     * @return remaining time in milliseconds, or 0 if done or not running
     */
    public double remainingMilliseconds() {
        if (!running) return 0;
        double remaining = (durationNanoseconds - (System.nanoTime() - startTimeNanoseconds)) / 1_000_000.0;
        return Math.max(0, remaining);
    }

    /**
     * Cancels the wait early.
     * <p>
     * After calling this, {@link #isDone()} will return {@code false} until
     * {@link #start(double)} is called again.
     */
    public void cancel() {
        running = false;
    }
}