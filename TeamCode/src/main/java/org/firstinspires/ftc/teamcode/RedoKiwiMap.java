package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

        // Set internal DCMotorEx PIDF Coefficients
        leftMotor.motorEx.setVelocityPIDFCoefficients(2.0, 3.0, 0.0, 0.0);
        rightMotor.motorEx.setVelocityPIDFCoefficients(2.0, 3.0, 0.0, 0.0);
        backMotor.motorEx.setVelocityPIDFCoefficients(2.0, 3.0, 0.0, 0.0);
    }

    /**
     * Drives the robot field oriented
     *
     * @param power The speed of the Robot [-1.0, 1.0]
     * @param angle the angle the robot drives at, forward is 180 since y is inverted
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
     * Goes to coordinates relative to its starting position
     *
     * @param coordinates the x and y of the robot
     */
    public void goToPosition(double[] coordinates, boolean autoAlign, double desiredAngle)
    {
        double[] xy = this.getXY();
        double dist = Math.sqrt(Math.pow(coordinates[0] - xy[0], 2 ) + Math.pow(coordinates[1] - xy[1], 2) );
        double theta = Math.toDegrees(Math.atan2(-(coordinates[1] - xy[1]), -(coordinates[0] - xy[0]))) - 90.0;
        double power = dist * Constants.P_P;

        if( power > 1.0 )
            power = 1.0;
        else if( power < -1.0 )
            power = -1.0;

        if( dist <= Constants.POSITION_THRESHOLD)
            this.drive(0.0, theta, 0.0, autoAlign, desiredAngle);
        else
            this.drive(power, theta, 0.0, autoAlign, desiredAngle);


    }

    /**
     * Corrects Robot Heading
     *
     * @return the turn speed to pass into drive()
     */
    public double headingCorrect(double desiredAngle)
    {
        double difference = desiredAngle - getHeading();
        if( difference < -180.0 )
            difference += 360.0;
        else if( difference >= 180.0 )
            difference -= 360.0;

        if( Math.abs(difference) < Constants.ANGLE_THRESHOLD )
            return 0.0;
        else if( difference >= 0.0 )
            return Constants.CORRECT_LIMITER * Math.pow(Math.sin(Math.toRadians(0.5 * difference)), 3.0/5);
        else
            return Constants.CORRECT_LIMITER * -Math.pow(Math.sin(Math.toRadians(0.5 * difference - 180.0)), 3.0/5);
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

    /**
     * Return the coordinates of the bot, assuming the robot's heading is constant
     *
     * @return the X and Y values relative to its starting position
     */
    public double[] getXY()
    {
        double back = backMotor.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double left = leftMotor.getCurrentPosition() * Constants.INCHES_PER_TICK;
        double right = rightMotor.getCurrentPosition() * Constants.INCHES_PER_TICK;

        return new double[]{-back, (left-right)/Math.sqrt(3.0)};
    }

    public double[] getPIDFCoeffs()
    {
        PIDFCoefficients coeffs = leftMotor.motorEx.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        return new double[]{coeffs.p, coeffs.i, coeffs.d, coeffs.f};
    }
}

