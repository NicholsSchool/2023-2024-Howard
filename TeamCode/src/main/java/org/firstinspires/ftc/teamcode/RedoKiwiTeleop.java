package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Kiwi Teleop Code
 */
@TeleOp(name="Teleop: New Kiwi", group="Iterative Opmode")
public class RedoKiwiTeleop extends OpMode implements Constants
{
    // Declare OpMode members.
    private RedoKiwiMap robot;
    private GamepadEx gamepad;
    private ElapsedTime runtime;
    private double desiredAngle;
    private boolean autoAlign;
    private double lastTurn;
    private double currentTurn;
    private double timeCounter;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize Runtime
        runtime = new ElapsedTime();

        // Initialize Gamepad
        gamepad = new GamepadEx(gamepad1);

        // Initialize Robot
        robot = new RedoKiwiMap();
        robot.init(hardwareMap);

        // Initialize Variables
        desiredAngle = 0.0;
        autoAlign = false;
        lastTurn = 0.1;
        currentTurn = 0.0;
        timeCounter = 0.0;

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        gamepad.readButtons();

        double x = gamepad.getLeftX();
        double y = gamepad.getLeftY();
        double power = Math.sqrt(x * x + y * y);
        double angle = Math.toDegrees(Math.atan2(y, x)) + 90.0;
        currentTurn = Constants.TURN_LIMITER *
                (gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if( currentTurn == 0.0 )
        {
            if( lastTurn != 0.0)
                timeCounter = runtime.time();
        }
        else
        {
            timeCounter = 0.0;
            autoAlign = false;
        }

        if( timeCounter != 0.0 && runtime.time() - timeCounter >= 0.1)
        {
            autoAlign = true;
            desiredAngle = robot.getHeading();
            timeCounter = 0.0;
        }

        if(gamepad.wasJustPressed(GamepadKeys.Button.Y))
            desiredAngle = 0.0;
        else if(gamepad.wasJustPressed(GamepadKeys.Button.A))
            desiredAngle = -180.0;
        else if(gamepad.wasJustPressed(GamepadKeys.Button.B))
            desiredAngle = 90.0;
        else if(gamepad.wasJustPressed(GamepadKeys.Button.X))
            desiredAngle = -90.0;

        robot.drive(power, angle, currentTurn, autoAlign, desiredAngle);

        lastTurn = currentTurn;

        // Show Telemetry
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Input Angle", angle);
        telemetry.addData("Robot Heading", robot.getHeading());
        telemetry.addData("Desired Angle", desiredAngle);
        telemetry.addData("Auto Align", autoAlign);
        telemetry.addData("Turn Power", currentTurn);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}
}




