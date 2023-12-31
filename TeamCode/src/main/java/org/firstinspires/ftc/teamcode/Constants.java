package org.firstinspires.ftc.teamcode;

/**
 * The Class Containing Constants for the Robot
 */
public interface Constants {
    /** Whether the drive wheels are inverted */
    public static final boolean INVERTED = true;

    /** Orientation of the IMU on the Robot */
    public static final float IMU_ANGLE = 150;


    /** Angle of the backMotor on the Robot */
    public static final double BACK_ANGLE = -180.0;

    /** Angle of the leftMotor on the Robot */
    public static final double LEFT_ANGLE = -60.0;

    /** Angle of the rightMotor on the Robot */
    public static final double RIGHT_ANGLE = 60.0;

    /** Diameter of the goBilda omni wheels on Howard */
    public static final double WHEEL_DIAMETER_INCHES = 2.83465;

    /** Ticks per Revolution of a 40:1 Rev Hex Motor */
    public static final int TICKS_PER_REV = 1120;

    /** Inches Traveled Per Encoder Tick */
    public static final double INCHES_PER_TICK = WHEEL_DIAMETER_INCHES * Math.PI / TICKS_PER_REV;

    /** Maximum power that our code applies to a motor */
    public static final double OVERALL_LIMITER = 0.9;

    /** Multiplier for joystick inputted turn speed */
    public static final double TURN_LIMITER = 0.45;

    /** Multiplier for automatic correction speed */
    public static final double CORRECT_LIMITER = 0.7;

    /** The threshold minimum value of error for goToPosition in inches */
    public static final double POSITION_THRESHOLD = 0.25;

    /** The P value for goToPosition */
    public static final double P_P = 0.1;

    /** The threshold minimum value of error for autoAlignment to occur in degrees */
    public static final double ANGLE_THRESHOLD = 0.3;
}


