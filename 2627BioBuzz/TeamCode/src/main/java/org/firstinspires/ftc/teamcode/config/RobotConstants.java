package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Centralized constants for all tunable robot parameters.
 * <p>
 * Every magic number used in subsystems, PID controllers, or OpModes should be
 * defined here. This makes tuning and maintenance much easier — you only need to
 * change values in one place instead of hunting through multiple files.
 */
public class RobotConstants {

    // =========================================================================
    // Hardware Configuration Feature Flags
    // =========================================================================

    /**
     * Set to {@code true} once the 4-motor mecanum drivetrain and 2-pod odometry
     * sensors are installed and configured on the Control Hub.
     * <p>
     * Set to {@code false} (in-between mode) to continue using the legacy
     * 2-motor tank drivetrain without requiring mecanum motors or odo pods.
     */
    public static final boolean MECANUM_ODO_INSTALLED = false;

    /**
     * Whether the 4 mecanum drive motors have their own encoder cables plugged in.
     * Set to {@code false} if encoder ports are used for odometry pods or drive
     * motors run without encoders (prevents false encoder faults on drive motors).
     */
    public static final boolean MECANUM_MOTORS_HAVE_ENCODERS = false;

    // =========================================================================
    // Telemetry / Dashboard / Driver Station Viewer
    // =========================================================================

    /** Master switch to disable ALL TelemetryServer streaming (laptop dashboard). */
    public static final boolean DISABLE_TELEMETRY = false;

    /** Telemetry server port. */
    public static final int TELEMETRY_SERVER_PORT = 8000;

    /** Driver Station telemetry transmission interval in milliseconds. */
    public static final int DS_TELEMETRY_INTERVAL_MS = 50;

    /** Battery voltage at or above which Driver Station shows healthy green. */
    public static final double DS_GOOD_BATTERY_VOLTAGE = 12.2;

    /** Battery voltage below which Driver Station shows warning/critical color. */
    public static final double DS_WARN_BATTERY_VOLTAGE = 11.4;

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

    /** Minimum ramp rate for DriveTrain (power per second). */
    public static final double DRIVETRAIN_MIN_RAMP_RATE = 0.01;

    /** Output bounds for DriveTrain clip function. */
    public static final double DRIVETRAIN_OUTPUT_MIN = -1.0;
    public static final double DRIVETRAIN_OUTPUT_MAX = 1.0;

    // Mecanum motor direction flags (flip if a wheel spins backward for positive power)
    public static final boolean MECANUM_REVERSE_FRONT_LEFT = true;
    public static final boolean MECANUM_REVERSE_FRONT_RIGHT = false;
    public static final boolean MECANUM_REVERSE_BACK_LEFT = true;
    public static final boolean MECANUM_REVERSE_BACK_RIGHT = false;

    // =========================================================================
    // Two-Wheel Odometry + IMU Configuration (inches & degrees)
    // =========================================================================
    // Robot coordinate convention:
    //   +X = Robot Forward
    //   +Y = Robot Left
    //   +Heading = Counter-Clockwise (CCW)
    //
    // Measure each odometry wheel center relative to the robot's tracking center:
    //   - X offset: positive = forward of center, negative = behind center
    //   - Y offset: positive = left of center, negative = right of center

    /** Encoder ticks per full revolution of each odometry dead wheel. */
    public static final double ODOMETRY_TICKS_PER_REV = 2000.0;

    /** Diameter of the odometry dead wheels in inches (e.g., 32mm = 1.25984, 48mm = 1.88976). */
    public static final double ODOMETRY_WHEEL_DIAMETER_INCHES = 32.0 / 25.4;

    /** Linear travel in inches per encoder tick for the forward odometry pod. */
    public static final double ODOMETRY_FORWARD_INCHES_PER_TICK =
            (Math.PI * ODOMETRY_WHEEL_DIAMETER_INCHES) / ODOMETRY_TICKS_PER_REV;

    /** Linear travel in inches per encoder tick for the strafe odometry pod. */
    public static final double ODOMETRY_STRAFE_INCHES_PER_TICK =
            (Math.PI * ODOMETRY_WHEEL_DIAMETER_INCHES) / ODOMETRY_TICKS_PER_REV;

    /**
     * Direction sign for the forward odometry pod (+1.0 or -1.0).
     * Must produce positive ticks when the robot moves FORWARD (+X).
     */
    public static final double ODOMETRY_FORWARD_POD_DIRECTION = 1.0;

    /**
     * Direction sign for the strafe odometry pod (+1.0 or -1.0).
     * Must produce positive ticks when the robot moves LEFT (+Y).
     */
    public static final double ODOMETRY_STRAFE_POD_DIRECTION = 1.0;

    /** Forward pod X position relative to robot center (inches, + = forward). */
    public static final double ODOMETRY_FORWARD_POD_X_INCHES = 0.0;

    /**
     * Forward pod Y position relative to robot center (inches, + = left, - = right).
     * Used to cancel forward-pod rotation coupling during turns.
     */
    public static final double ODOMETRY_FORWARD_POD_Y_INCHES = 0.0;

    /**
     * Strafe pod X position relative to robot center (inches, + = forward, - = backward).
     * Used to cancel strafe-pod rotation coupling during turns.
     */
    public static final double ODOMETRY_STRAFE_POD_X_INCHES = 0.0;

    /** Strafe pod Y position relative to robot center (inches, + = left). */
    public static final double ODOMETRY_STRAFE_POD_Y_INCHES = 0.0;

    /** Small-angle threshold (radians) for Taylor approximation in pose exponential. */
    public static final double ODOMETRY_SMALL_ANGLE_THRESHOLD = 1e-6;

    /** Control Hub logo orientation on the robot for the built-in IMU. */
    public static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_FACING_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;

    /** Control Hub USB port orientation on the robot for the built-in IMU. */
    public static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_FACING_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // =========================================================================
    // FlyWheel Constants
    // =========================================================================

    /** Default target speed for the flywheel (encoder ticks per second). */
    public static final double FLYWHEEL_DEFAULT_TARGET_SPEED = 200.0;

    /** FlyWheel PID proportional gain. */
    public static final double FLYWHEEL_P = 0.0002;

    /** FlyWheel PID integral gain. */
    public static final double FLYWHEEL_I = 0.00002;

    /** FlyWheel PID derivative gain. */
    public static final double FLYWHEEL_D = 0.00002;

    /** FlyWheel PID feedforward gain. (1 / Max Speed Motor, No Load) */
    public static final double FLYWHEEL_F = 0.0020;

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

    /** Grace period when spinning up to normal before judging super-mode boost (ms). */
    public static final long INTAKE_SPINUP_GRACE_MS = 250;

    /** Grace period after entering STOPPED so coasting doesn't re-trigger NORMAL (ms). */
    public static final long INTAKE_STOP_COAST_GRACE_MS = 400;

    /** Maximum time to reverse before returning to normal (ms). */
    public static final long INTAKE_REVERSE_TIME_MS = 200;

    /** Time stopped before going back to idle (ms). */
    public static final long INTAKE_STOP_TIME_MS = 5000;

    // =========================================================================
    // Fault Detection & Motor Error Utility Thresholds
    // =========================================================================

    /** Minimum interval between velocity delta samples for non-DcMotorEx motors (ns). */
    public static final long VELOCITY_MIN_SAMPLE_NANOSECONDS = 5_000_000L;

    /** Maximum allowed PID output before considering it saturated (0.0 to 1.0). */
    public static final double PID_SATURATION_THRESHOLD = 0.95;

    /** Time in seconds before declaring a motor stalled (no position change). */
    public static final double MOTOR_STALL_TIMEOUT_SECONDS = 1.0;

    /** Maximum allowed difference between target and actual position (degrees). */
    public static final double POSITION_ERROR_THRESHOLD_DEGREES = 10.0;

    /** Minimum battery voltage before warning (volts). */
    public static final double LOW_BATTERY_VOLTAGE_THRESHOLD = 11.0;

    /** Overcurrent threshold for DcMotorEx motors (Amps). */
    public static final double MOTOR_OVERCURRENT_AMPS = 8.5;

    /** Duration (ms) above overcurrent threshold before reporting OVERCURRENT. */
    public static final long MOTOR_OVERCURRENT_TIMEOUT_MS = 400;

    /** Power threshold above which to check for reversed encoder polarity (0.0 to 1.0). */
    public static final double MOTOR_REVERSED_ENCODER_POWER_THRESHOLD = 0.25;

    /** Speed magnitude (ticks/sec) opposite to power sign needed to flag reversed encoder. */
    public static final double MOTOR_REVERSED_ENCODER_SPEED_THRESHOLD = 100.0;

    /** Duration (ms) of opposite-sign speed before flagging REVERSED_ENCODER. */
    public static final long MOTOR_REVERSED_ENCODER_TIMEOUT_MS = 500;

    /** Duration (ms) of continuous PID saturation before reporting PID_SATURATED. */
    public static final long MOTOR_PID_SATURATION_TIMEOUT_MS = 750;

    /** Duration (ms) of continuous speed error before reporting SPEED_ERROR. */
    public static final long MOTOR_SPEED_ERROR_TIMEOUT_MS = 750;

    // =========================================================================
    // DriveTrain Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double DRIVETRAIN_ENCODER_FAULT_POWER_THRESHOLD = 0.15;

    /** Time (ms) with no position change while powered before declaring encoder fault. */
    public static final long DRIVETRAIN_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double DRIVETRAIN_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double DRIVETRAIN_STALL_SPEED_THRESHOLD = 50.0;

    // =========================================================================
    // Intake Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double INTAKE_ENCODER_FAULT_POWER_THRESHOLD = 0.15;

    /** Time (ms) with no position change while powered before declaring encoder fault. */
    public static final long INTAKE_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double INTAKE_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double INTAKE_STALL_SPEED_THRESHOLD = 50.0;

    // =========================================================================
    // FlyWheel Fault Detection Thresholds
    // =========================================================================

    /** Power threshold above which to check for encoder faults (0.0 to 1.0). */
    public static final double FLYWHEEL_ENCODER_FAULT_POWER_THRESHOLD = 0.15;

    /** Time (ms) with no position change while powered before declaring encoder fault. */
    public static final long FLYWHEEL_ENCODER_FAULT_TIMEOUT_MS = 500;

    /** Power threshold above which to check for motor stall (0.0 to 1.0). */
    public static final double FLYWHEEL_STALL_POWER_THRESHOLD = 0.2;

    /** Speed threshold (ticks/sec) below which motor is considered stalled. */
    public static final double FLYWHEEL_STALL_SPEED_THRESHOLD = 50.0;

    /** Speed error threshold as fraction of target (e.g., 0.2 = 20%). */
    public static final double FLYWHEEL_SPEED_ERROR_THRESHOLD_FRACTION = 0.2;

    // =========================================================================
    // Autonomous Constants
    // =========================================================================

    /** Drive power for AutoLeave routine (0.0 to 1.0). */
    public static final double AUTO_LEAVE_DRIVE_POWER = 0.2;

    /** Wait duration for AutoLeave routine (milliseconds). */
    public static final long AUTO_LEAVE_WAIT_MILLISECONDS = 1000;

    // =========================================================================
    // PID Controller Default Constants
    // =========================================================================

    /** Default minimum output for PID controllers. */
    public static final double PID_DEFAULT_OUTPUT_MIN = -1.0;

    /** Default maximum output for PID controllers. */
    public static final double PID_DEFAULT_OUTPUT_MAX = 1.0;

    // =========================================================================
    // Emergency Stop (E-Stop) & Auto E-Stop Constants
    // =========================================================================

    /** Both triggers must exceed this value (0.0 to 1.0) to trigger global E-Stop. */
    public static final double ESTOP_TRIGGER_THRESHOLD = 0.85;

    /** Duration (ms) both triggers must be held simultaneously to trigger E-Stop. */
    public static final long ESTOP_TRIGGER_HOLD_MS = 150;

    /** Duration (ms) BACK button must be held to clear/reset E-Stop. */
    public static final long ESTOP_RESET_HOLD_MS = 500;

    /** Master switch for automatic FlyWheel E-Stop on critical faults. */
    public static final boolean AUTO_ESTOP_FLYWHEEL_ENABLED = true;

    /** Auto E-Stop FlyWheel when motor stall is detected. */
    public static final boolean AUTO_ESTOP_FLYWHEEL_ON_STALL = true;

    /** Auto E-Stop FlyWheel when encoder fault is detected (prevents PID runaway). */
    public static final boolean AUTO_ESTOP_FLYWHEEL_ON_ENCODER_FAULT = true;

    /** Auto E-Stop FlyWheel when overcurrent is detected (DcMotorEx). */
    public static final boolean AUTO_ESTOP_FLYWHEEL_ON_OVERCURRENT = true;

    /** Master switch for automatic Intake E-Stop on critical faults. */
    public static final boolean AUTO_ESTOP_INTAKE_ENABLED = true;

    /** Auto E-Stop Intake when jam recovery (reverse + super retry) fails. */
    public static final boolean AUTO_ESTOP_INTAKE_ON_JAM_FAIL = true;

    /** Auto E-Stop Intake when encoder fault is detected. */
    public static final boolean AUTO_ESTOP_INTAKE_ON_ENCODER_FAULT = true;

    /** Auto E-Stop Intake when overcurrent is detected (DcMotorEx). */
    public static final boolean AUTO_ESTOP_INTAKE_ON_OVERCURRENT = true;

    /** Auto E-Stop DriveTrain when sustained overcurrent is detected (DcMotorEx). */
    public static final boolean AUTO_ESTOP_DRIVETRAIN_ON_OVERCURRENT = true;

    /** Auto E-Stop DriveTrain on wheel stall (keep false during matches for pushing/defense). */
    public static final boolean AUTO_ESTOP_DRIVETRAIN_ON_STALL = false;
}
