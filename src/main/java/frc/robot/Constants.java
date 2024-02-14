/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Physical peripheral connections --> software port number
    public static final int COMPRESSOR_ID = 0;
    public static final int PCM_ID = 0;
    public static final int DRIVETRAIN_FRONTLEFT_MOTOR = 0;
    public static final int REAR_INTAKE_DSOL_FORWARD_CHANNEL = 0;
    public static final int TURRET_LED_CONTROLLER_PWM_PORT = 0;
    public static final int DRIVETRAIN_REARLEFT_MOTOR = 1;
    public static final int REAR_INTAKE_DSOL_REVERSE_CHANNEL = 1;
    public static final int SHOOTER_ENCODER_CHANNEL_A = 2;
    public static final int DRIVETRAIN_FRONTRIGHT_MOTOR = 2;
    public static final int FRONT_INTAKE_DSOL_FORWARD_CHANNEL = 2;
    public static final int SHOOTER_ENCODER_CHANNEL_B = 3;
    public static final int DRIVETRAIN_REARRIGHT_MOTOR = 3;
    public static final int FRONT_INTAKE_DSOL_REVERSE_CHANNEL = 3;
    public static final int AGITATOR_MOTOR = 4;
    public static final int SHOOTER_SOL_CHANNEL = 4;
    public static final int FRONT_INTAKE_MOTOR = 5;
    public static final int CONVEYOR_BACK_MOTOR = 6;
    public static final int CONVEYOR_FRONT_MOTOR = 7;
    public static final int REAR_INTAKE_MOTOR = 10;
    public static final int TURRET_MOTOR = 11;
    public static final int SHOOTER_LEFT_MOTOR = 12;
    public static final int SHOOTER_RIGHT_MOTOR = 13;

    // Constants for Drivetrain subsystem
    public static final Gains DRIVETRAIN_kGains = new Gains(0.02, 0.0, 0.0); // P, I, D
    public static final double DRIVETRAIN_kUnitsPerRevolution = 2048;
    public static final double DRIVETRAIN_GEAR_RATIO = 10.75;
    public static final double DRIVETRAIN_WHEEL_CIRCUMFERENCE = Math.PI * 6; // Pi times the diameter of the wheel
    public static final double DRIVETRAIN_CLOSED_LOOP_TOLERANCE = 0.5; // inches
    /**
     * Free speed of Falcon 500 is 6380 RPM --> ~21777 sensor units per 100ms
     * Peak power from Falcon 500 is attainable at 3190 RPM --> ~10888 sensor units per 100ms
     * !!! Both speeds are based off of a 2048 quad encoder !!!
     */
    public static final int DRIVETRAIN_PEAK_SPEED = 21777 * 2; // max speed scaled by 3
    public static final int DRIVETRAIN_POWER_EFFICIENCY_SPEED = 10888;
    public static final double DRIVETRAIN_RAMP_TIME = 0.75; // seconds
    public static final double DRIVETRAIN_BASELINE_DRIVE_DISTANCE = 84.0; // inches

    // Constants for Intake subsystem
    public static final double INTAKE_WHEEL_SPEED = 1.0;
    
    // Constants for Agitator subsystem
    public static final double AGITATOR_SPEED = 0.8;

    // Constants for Conveyor subsystem
    public static final double CONVEYOR_SPEED = 0.7;

    // Constants for Turret subsystem
    public static final Gains TURRET_kGains = new Gains(-0.02, 0.0, -0.001); // P, I, D
    public static final double TURRET_PEAK_OUTPUT = 1.0;

    // Constants for Shooter Subsystem
    public static final int SHOOTER_kPIDLoopIdx = 0;
    public static final int SHOOTER_kTimeoutMs = 0;
    public static final Gains SHOOTER_kGains = new Gains(1.0, 0.0, 0.0); // P, I, D
    public static final double SHOOTER_PEAK_OUTPUT = 1.0;
    public static final double SHOOTER_GEAR_RATIO = 2.0;
    /**
     * Free speed of 775 Pro is 18730 RPM --> ~63931 sensor units per 100ms
     * !!! Speed based off of a 2048 quad encoder !!!
     */
    public static final int SHOOTER_PEAK_SPEED = (int) (63931 / SHOOTER_GEAR_RATIO);
    public static final int SHOOTER_RESTING_SPEED = (int) (SHOOTER_PEAK_SPEED * 0.2);
    public static final int SHOOTER_VELOCITY_TOLERANCE = (int) (SHOOTER_PEAK_SPEED * 0.05);

    // Constants for Vision System
    public static final double VISION_SHOOTER_HOOD_SWITCH_DISTANCE = 120.00; // inches
    public static final double VISION_HEIGHT_OF_POWER_PORT_TARGET = 98.19; // inches
    public static final double VISION_HEIGHT_OF_LIMELIGHT = 37.5; // inches
    public static final double VISION_ANGLE_OF_LIMELIGHT = 20.228; // degrees
    public static final double VISION_ANGLE_TOLERANCE = 5.0; // degrees

    // Constants for Blinkin LED Driver
    public static final double LED_COLOR_OFF = 0.0;
    public static final double LED_COLOR_RED = 0.61;
    public static final double LED_COLOR_ORANGE = 0.65;
    public static final double LED_COLOR_GREEN = 0.73;
    public static final double LED_COLOR_BLUE = 0.83;
    public static final double LED_COLOR_RED_LARSON_SCANNER = -0.35;
}
