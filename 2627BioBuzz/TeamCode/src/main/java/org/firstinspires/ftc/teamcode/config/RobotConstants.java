package org.firstinspires.ftc.teamcode.config;

/**
 * Centralized constants for all tunable robot parameters.
 * <p>
 * Every magic number used in subsystems, PID controllers, or OpModes should be
 * defined here. This makes tuning and maintenance much easier — you only need to
 * change values in one place instead of hunting through multiple files.
 */
public class RobotConstants {

    // =========================================================================
    // Gamepad Constants
    // =========================================================================

    /** Deadzone for gamepad sticks and triggers (0.0 to 1.0). */
    public static final double GAMEPAD_DEADZONE = 0.05;

    /** Speed multiplier for fast mode (full speed). */
    public static final double FAST_SPEED_MULTIPLIER = 1.0;

    /** Speed multiplier for slow/precision mode. */
    public static final double SLOW_SPEED_MULTIPLIER = 0.5;

    // =========================================================================
    // DriveTrain Constants
    // =========================================================================

    /** Acceleration rate for drive motors (power change per second). */
    public static final double DRIVETRAIN_ACCELERATION_RATE = 5.0;

    /** Deceleration rate for drive motors (power change per second). */
    public static final double DRIVETRAIN_DECELERATION_RATE = 100.0;

    /** Maximum delta time to process in update loop (seconds) — ignores loop hiccups. */
    public static final double DRIVETRAIN_MAX_DELTA_TIME_SECONDS = 0.1;

    // =========================================================================
    // FlyWheel Constants
    // =========================================================================

    /** Default target speed for the flywheel (encoder ticks per second). */
    public static final double FLYWHEEL_DEFAULT_TARGET_SPEED = 200.0;

    /** FlyWheel PID proportional gain. */
    public static final double FLYWHEEL_P = 0.0;

    /** FlyWheel PID integral gain. */
    public static final double FLYWHEEL_I = 0.0;

    /** FlyWheel PID derivative gain. */
    public static final double FLYWHEEL_D = 0.0;

    /** FlyWheel PID feedforward gain. */
    public static final double FLYWHEEL_F = 0.0;

    // =========================================================================
    // Intake Constants
    // =========================================================================

    /** Intake motor state: stopped. */
    public static final int INTAKE_MOTOR_STOPPED = 0;

    /** Intake motor state: idle (holding low speed). */
    public static final int INTAKE_MOTOR_IDLE = 1;

    /** Intake motor state: normal intake operation. */
    public static final int INTAKE_MOTOR_NORMAL = 2;

    /** Intake motor state: super mode (high speed for pushing through jams). */
    public static final int INTAKE_MOTOR_SUPER = 3;

    /** Intake motor state: reverse (for clearing jams). */
    public static final int INTAKE_MOTOR_REVERSE = 4;

    // Speeds
    /** Normal intake motor speed (-1.0 to 1.0). */
    public static final double INTAKE_MOTOR_SPEED = 0.5;

    /** Super mode intake motor speed (-1.0 to 1.0). */
    public static final double INTAKE_MOTOR_SUPER_SPEED = 1.0;

    /** Idle intake motor speed (-1.0 to 1.0). */
    public static final double INTAKE_MOTOR_IDLE_SPEED = 0.1;

    /** Reverse intake motor speed (-1.0 to 1.0). */
    public static final double INTAKE_MOTOR_REVERSE_SPEED = -0.5;

    // Speed thresholds
    /** Minimum speed to consider motor started (ticks/sec). */
    public static final int INTAKE_MOTOR_START_SPEED = 5;

    /** Speed below which motor is considered stalled (ticks/sec). */
    public static final int INTAKE_MOTOR_STALL_SPEED = 50;

    /** Speed threshold to enter super mode (ticks/sec). */
    public static final int INTAKE_MOTOR_SUPER_START_SPEED = 500;

    // Idle detection
    /** Time window for stability detection (ms). */
    public static final long INTAKE_STABLE_TIME_MS = 5000;

    /** Maximum average speed change between loops for stability (ticks/sec). */
    public static final double INTAKE_STABLE_THRESHOLD = 8.0;

    /** Time to let motor settle after entering idle (ms). */
    public static final long INTAKE_IDLE_SETTLE_MS = 1000;

    /** Speed deviation from idle average that exits idle (ticks/sec). */
    public static final double INTAKE_IDLE_EXIT_THRESHOLD = 50.0;

    // Stall recovery
    /** Grace period after state change before stall checks (ms). */
    public static final long INTAKE_STATE_GRACE_MS = 50;

    /** Maximum time to reverse before returning to normal (ms). */
    public static final long INTAKE_REVERSE_TIME_MS = 200;

    /** Time stopped before going back to idle (ms). */
    public static final long INTAKE_STOP_TIME_MS = 5000;

    // =========================================================================
    // Fault Detection Thresholds
    // =========================================================================

    /** Maximum allowed PID output before considering it saturated (0.0 to 1.0). */
    public static final double PID_SATURATION_THRESHOLD = 0.95;

    /** Time in seconds before declaring a motor stalled (no position change). */
    public static final double MOTOR_STALL_TIMEOUT_SECONDS = 1.0;

    /** Maximum allowed difference between target and actual position (degrees). */
    public static final double POSITION_ERROR_THRESHOLD_DEGREES = 10.0;

    /** Minimum battery voltage before warning (volts). */
    public static final double LOW_BATTERY_VOLTAGE_THRESHOLD = 11.0;

    // =========================================================================
    // DriveTrain Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD = 0.1;

    /** Time (ms) with no position change before declaring encoder fault. */
    public static final long DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double DRIVETRAIN_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double DRIVETRAIN_STALL_SPEED_THRESHOLD = 50.0;

    // =========================================================================
    // Intake Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double INTAKE_ENCODER_FAULT_POWER_THRESHOLD = 0.1;

    /** Time (ms) with no position change before declaring encoder fault. */
    public static final long INTAKE_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double INTAKE_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double INTAKE_STALL_SPEED_THRESHOLD = 50.0;

    // =========================================================================
    // FlyWheel Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double FLYWHEEL_ENCODER_FAULT_POWER_THRESHOLD = 0.1;

    /** Time (ms) with no position change before declaring encoder fault. */
    public static final long FLYWHEEL_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double FLYWHEEL_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double FLYWHEEL_STALL_SPEED_THRESHOLD = 50.0;

    /** Speed error threshold as fraction of target (e.g., 0.2 = 20%). */
    public static final double FLYWHEEL_SPEED_ERROR_THRESHOLD_FRACTION = 0.2;
}