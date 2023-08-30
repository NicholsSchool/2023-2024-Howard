package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * The Class defining the Robot. 0 degrees is forward on the bot, positive is clockwise.
 * Positive wheel spin turns robot to the right.
 */
public class RedoKiwiMap implements Constants {
    private BHI260IMU imu;
    private MotorEx leftMotor;
    private MotorEx rightMotor;
    private MotorEx backMotor;

    /**
     * Initialize the RobotMap
     *
     * @param hwMap the HardwareMap object to be passed in
     */
    public void init( HardwareMap hwMap )
    {
        // Instantiating IMU Parameters, setting angleUnit...
        BHI260IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                Constants.IMU_ANGLE,
                                0,
                                0,
                                0  // acquisitionTime, not used
                        )
                )
        );

        // Defining and Initializing IMU... Initializing it with the above Parameters...
        imu = hwMap.get( BHI260IMU.class, "imu" );
        imu.initialize( parameters );
        imu.resetYaw();

        // Initialize Motors
        leftMotor = new MotorEx(hwMap, "leftMotor");
        rightMotor = new MotorEx(hwMap, "rightMotor");
        backMotor = new MotorEx(hwMap, "backMotor");

        // Invert Motors
        leftMotor.setInverted(Constants.INVERTED);
        rightMotor.setInverted(Constants.INVERTED);
        backMotor.setInverted(Constants.INVERTED);

        // Set Motor Zero Power Behavior
        leftMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        backMotor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        // Set Motor RunMode to Raw Power
        leftMotor.setRunMode(MotorEx.RunMode.RawPower);
        rightMotor.setRunMode(MotorEx.RunMode.RawPower);
        backMotor.setRunMode(MotorEx.RunMode.RawPower);

        // Reset Motor Encoders
        leftMotor.stopAndResetEncoder();
        rightMotor.stopAndResetEncoder();
        backMotor.stopAndResetEncoder();

        // Set internal DCMotorEx object Runmode to use encoder
        leftMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backMotor.motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot field oriented
     *
     * @param power The speed of the Robot [-1.0, 1.0]
     * @param angle the angle the robot drives at
     * @param turn the turn speed of the Robot [-1.0, 1.0]
     * @param autoAlign whether to correct the robot's heading
     * @param desiredAngle what angle to correct to

     */
    public void drive( double power, double angle, double turn, boolean autoAlign, double desiredAngle ) {
        if( autoAlign )
            turn = headingCorrect(desiredAngle);

        double driveCoeff = Constants.OVERALL_LIMITER - Math.abs(turn);
        leftMotor.set(turn + (driveCoeff * power * Math.sin(Math.toRadians(angle + Constants.LEFT_ANGLE + getHeading()))));
        rightMotor.set(turn + (driveCoeff * power * Math.sin(Math.toRadians(angle + Constants.RIGHT_ANGLE + getHeading()))));
        backMotor.set(turn + (driveCoeff * power * Math.sin((Math.toRadians(angle + Constants.BACK_ANGLE + getHeading())))));
    }

    /**
     * Corrects Robot Heading
     *
     * @return the turn speed to pass into drive()
     */
    public double headingCorrect(double desiredAngle)
    {
        double difference = desiredAngle - getHeading();
        if( difference < -180 )
            difference += 360;
        else if( difference >= 180 )
            difference -= 360;
        if( Math.abs(difference) < Constants.ANGLE_THRESHOLD )
            return 0.0;
        else if( difference > 90.0)
            return Constants.STRAIGHTEN_LIMITER * (Math.sin(Math.toRadians(0.5 * difference + 45.0)));
        else if( difference < -90.0 )
            return Constants.STRAIGHTEN_LIMITER * (Math.sin(Math.toRadians(0.5 * difference - 45.0)));
        return Constants.STRAIGHTEN_LIMITER * (Math.sin(Math.toRadians(difference)));
    }

    /**
     * The current Heading of the Robot
     *
     * @return the current Heading in the range of [-180, 180)
     */
    public double getHeading()
    {
        return -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}

