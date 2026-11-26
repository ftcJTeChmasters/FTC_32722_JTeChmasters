package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.HardwareNames;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.robot.debug.TelemetryServer;

/**
 * Aimer subsystem with servo control and automatic ball size detection via color sensor.
 * <p>
 * The aimer positions a guide/flap near the flywheel to direct balls. It has two positions:
 * - Large balls (blue/red): wider opening
 * - Small balls (yellow): narrower opening
 * <p>
 * A color sensor mounted just before the flywheel detects the incoming ball color.
 * The robot knows its alliance color (set via gamepad, not code change) so it won't
 * shoot opponent-colored balls.
 */
public class Aimer {

    private final Servo aimerServo;
    private final ColorSensor colorSensor;

    /** Current target position of the aimer servo. */
    private double targetPosition = RobotConstants.AIMER_POSITION_LARGE_BALLS;

    /** Last detected ball type from color sensor. */
    public enum BallType {
        UNKNOWN,
        LARGE_BLUE,
        LARGE_RED,
        SMALL_YELLOW
    }
    private BallType lastDetectedBallType = BallType.UNKNOWN;

    /** Current HSV values from color sensor. */
    private float[] hsvValues = new float[3];

    /** Whether auto-detection is enabled (default true). */
    private boolean autoDetectEnabled = true;

    /** Whether the aimer is currently moving to a new position. */
    private boolean isMoving = false;

    public static final String SUBSYSTEM_KEY = "aimer";

    public Aimer(HardwareMap hardwareMap) {
        aimerServo = hardwareMap.get(Servo.class, HardwareNames.AIMER_SERVO);
        colorSensor = hardwareMap.get(ColorSensor.class, HardwareNames.COLOR_SENSOR);

        // Initialize servo to large ball position
        aimerServo.setPosition(RobotConstants.AIMER_POSITION_LARGE_BALLS);
        targetPosition = RobotConstants.AIMER_POSITION_LARGE_BALLS;

        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "position", targetPosition);
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "ballType", "UNKNOWN");
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "autoDetect", autoDetectEnabled);
    }

    /**
     * Sets the aimer to the large ball position (for blue/red balls).
     */
    public void setLargeBallPosition() {
        targetPosition = RobotConstants.AIMER_POSITION_LARGE_BALLS;
        aimerServo.setPosition(targetPosition);
        isMoving = true;
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "position", targetPosition);
    }

    /**
     * Sets the aimer to the small ball position (for yellow balls).
     */
    public void setSmallBallPosition() {
        targetPosition = RobotConstants.AIMER_POSITION_SMALL_BALLS;
        aimerServo.setPosition(targetPosition);
        isMoving = true;
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "position", targetPosition);
    }

    /**
     * Toggles between large and small ball positions.
     */
    public void togglePosition() {
        if (targetPosition == RobotConstants.AIMER_POSITION_LARGE_BALLS) {
            setSmallBallPosition();
        } else {
            setLargeBallPosition();
        }
    }

    /**
     * Sets the aimer servo position directly (0.0 to 1.0).
     *
     * @param position servo position (0.0 to 1.0)
     */
    public void setPosition(double position) {
        targetPosition = Math.max(0.0, Math.min(1.0, position));
        aimerServo.setPosition(targetPosition);
        isMoving = true;
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "position", targetPosition);
    }

    /**
     * Enables or disables automatic ball type detection from color sensor.
     *
     * @param enabled true to enable auto-detection
     */
    public void setAutoDetectEnabled(boolean enabled) {
        this.autoDetectEnabled = enabled;
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "autoDetect", enabled);
    }

    /** Returns true if auto-detection is enabled. */
    public boolean isAutoDetectEnabled() {
        return autoDetectEnabled;
    }

    /** Returns the last detected ball type. */
    public BallType getLastDetectedBallType() {
        return lastDetectedBallType;
    }

    /** Returns the current target servo position. */
    public double getTargetPosition() {
        return targetPosition;
    }

    /** Returns the current HSV values from the color sensor. */
    public float[] getHsvValues() {
        return hsvValues.clone();
    }

    /** Returns true if the aimer servo is currently at the large ball position. */
    public boolean isAtLargeBallPosition() {
        return Math.abs(targetPosition - RobotConstants.AIMER_POSITION_LARGE_BALLS) < 0.05;
    }

    /** Returns true if the aimer servo is currently at the small ball position. */
    public boolean isAtSmallBallPosition() {
        return Math.abs(targetPosition - RobotConstants.AIMER_POSITION_SMALL_BALLS) < 0.05;
    }

    /**
     * Updates the color sensor reading and performs auto-detection if enabled.
     * Call this once per loop from the OpMode.
     */
    public void update() {
        // Read color sensor (RGB to HSV conversion)
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        // Convert RGB to HSV (0-360, 0-1, 0-1)
        float[] hsv = rgbToHsv(red, green, blue);
        hsvValues = hsv;

        // Detect ball type based on HSV hue
        BallType detectedType = detectBallType(hsv[0], hsv[1], hsv[2]);
        lastDetectedBallType = detectedType;

        // Auto-adjust aimer position based on detected ball type
        if (autoDetectEnabled && detectedType != BallType.UNKNOWN) {
            boolean shouldBeLarge = (detectedType == BallType.LARGE_BLUE || detectedType == BallType.LARGE_RED);

            // Check team color restriction: blue alliance doesn't shoot red balls, red alliance doesn't shoot blue balls
            boolean isOpponentColor = (RobotConstants.TEAM_COLOR_BLUE && detectedType == BallType.LARGE_RED)
                    || (!RobotConstants.TEAM_COLOR_BLUE && detectedType == BallType.LARGE_BLUE);

            if (shouldBeLarge && !isOpponentColor) {
                if (!isAtLargeBallPosition()) {
                    setLargeBallPosition();
                }
            } else if (detectedType == BallType.SMALL_YELLOW) {
                if (!isAtSmallBallPosition()) {
                    setSmallBallPosition();
                }
            }
            // If it's opponent color large ball, don't adjust (keep current position)
        }

        // Report telemetry
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "ballType", detectedType.name());
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "hue", hsv[0]);
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "saturation", hsv[1]);
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "value", hsv[2]);
        TelemetryServer.getInstance().setSubsystemTelemetry("aimer", "teamColorBlue", RobotConstants.TEAM_COLOR_BLUE);
    }

    /**
     * Detects ball type from HSV values.
     *
     * @param hue        hue (0-360)
     * @param saturation saturation (0-1)
     * @param value      value/brightness (0-1)
     * @return detected ball type
     */
    private BallType detectBallType(float hue, float saturation, float value) {
        // Need minimum saturation and value for reliable color detection
        if (saturation < RobotConstants.COLOR_MIN_SATURATION || value < RobotConstants.COLOR_MIN_VALUE) {
            return BallType.UNKNOWN;
        }

        // Check for BLUE (large ball)
        if (hue >= RobotConstants.COLOR_BLUE_HUE_MIN && hue <= RobotConstants.COLOR_BLUE_HUE_MAX) {
            return BallType.LARGE_BLUE;
        }

        // Check for RED (large ball) - wraps around 0/360
        if ((hue >= RobotConstants.COLOR_RED_HUE_MIN_1 && hue <= RobotConstants.COLOR_RED_HUE_MAX_1)
                || (hue >= RobotConstants.COLOR_RED_HUE_MIN_2 && hue <= RobotConstants.COLOR_RED_HUE_MAX_2)) {
            return BallType.LARGE_RED;
        }

        // Check for YELLOW (small ball)
        if (hue >= RobotConstants.COLOR_YELLOW_HUE_MIN && hue <= RobotConstants.COLOR_YELLOW_HUE_MAX) {
            return BallType.SMALL_YELLOW;
        }

        return BallType.UNKNOWN;
    }

    /**
     * Converts RGB values (0-255) to HSV (hue 0-360, saturation 0-1, value 0-1).
     */
    private float[] rgbToHsv(int r, int g, int b) {
        float rf = r / 255.0f;
        float gf = g / 255.0f;
        float bf = b / 255.0f;

        float max = Math.max(rf, Math.max(gf, bf));
        float min = Math.min(rf, Math.min(gf, bf));
        float delta = max - min;

        float h = 0;
        float s = (max == 0) ? 0 : delta / max;
        float v = max;

        if (delta != 0) {
            if (max == rf) {
                h = 60 * (((gf - bf) / delta) % 6);
            } else if (max == gf) {
                h = 60 * (((bf - rf) / delta) + 2);
            } else {
                h = 60 * (((rf - gf) / delta) + 4);
            }
        }
        if (h < 0) h += 360;

        return new float[]{h, s, v};
    }

    /** Sets the team color (called from gamepad, not hardcoded). */
    public static void setTeamColorBlue(boolean isBlue) {
        RobotConstants.TEAM_COLOR_BLUE = isBlue;
    }

    /** Returns true if the current team color is blue alliance. */
    public static boolean isTeamColorBlue() {
        return RobotConstants.TEAM_COLOR_BLUE;
    }
}