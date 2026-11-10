package org.firstinspires.ftc.teamcode.config;

public class RobotConstants {
    // Controlls
    public static final double  GAMEPAD_DEADZONE                =   0.05; // Deadzone for gamepad sticks and triggers

    // Intake
        // Internal References
    public static final int     INTAKE_MOTOR_STOPED             = 0;
    public static final int     INTAKE_MOTOR_IDLE               = 1;
    public static final int     INTAKE_MOTOR_NORMAL             = 2;
    public static final int     INTAKE_MOTOR_SUPER              = 3;
        // Speeds
    public static final double  INTAKE_MOTOR_SPEED              =   0.5; 
    public static final double  INTAKE_MOTOR_SUPER_SPEED        =   1.0;
    public static final double  INTAKE_MOTOR_IDLE_SPEED         =   0.1;
        // Speed thresholds
    public static final int     INTAKE_MOTOR_START_SPEED        =   5;
    public static final int     INTAKE_MOTOR_STALL_SPEED        =   10;
    public static final int     INTAKE_MOTOR_SUPER_START_SPEED  =   50;
        // Idle detection
    public static final long    INTAKE_STABLE_TIME_MS           =   5000; // Window the speed must be stable for
    public static final double  INTAKE_STABLE_THRESHOLD         =   4.0;  // Max avg change (ticks/s) between loops
    public static final long    INTAKE_IDLE_SETTLE_MS           =   1000; // Time to let the motor settle after entering idle
    public static final double  INTAKE_IDLE_EXIT_THRESHOLD      =   15.0; // Deviation (ticks/s) from idle average that leaves idle
}