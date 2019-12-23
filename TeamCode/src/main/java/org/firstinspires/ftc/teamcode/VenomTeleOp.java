package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VenomTeleOp", group = "6209")
public class VenomTeleOp extends OpMode
{
    VenomRobot robot = new VenomRobot();
    double intakePower = 1;
    double drivePower = 1;


    @Override
    public void init()
    {
        robot.init(hardwareMap, telemetry, false);
        telemetry.addData("Initialization complete", "");
        telemetry.update();

    }

    public void start()
    {
        // add later
    }

    public void stop()
    {
        // add later
    }

    public void loop()
    {
        doDrive();
        doFoundation();
        // doIntake();
        doOutput();
    }

    void doDrive()
    {
        slowDown();

        double forward = -gamepad1.left_stick_y * drivePower;
        double strafe = -gamepad1.left_stick_x * drivePower;
        double rotate = gamepad1.right_stick_x * drivePower;

        robot.driveTrain.arcadeDrive(forward, strafe, rotate);
        robot.driveTrain.arcadeDrive(forward, strafe, rotate);

    }

    public void slowDown() {

        if(gamepad1.right_trigger > 0.7)
            drivePower = 0.5;

        else if (gamepad1.left_trigger > 0.7)
            drivePower = 0.25;

        else
            drivePower = 1;
    }

    /** public void doIntake()
    {
        if(gamepad2.right_bumper)
            robot.intake.setPower(intakePower);
        else if (gamepad2.left_bumper)
            robot.intake.setPower(-intakePower);
        else
            robot.intake.setPower(0);
    }**/

    public void doOutput()
    {
        // lift --> dpad
        if (gamepad2.dpad_up)
        {
            robot.output.startMoveLiftUp();
        }
        else if (gamepad2.dpad_down)
        {
            robot.output.startMoveLiftDown();
        }
        else
            {
            robot.output.stopLift();
        }

        if (gamepad2.x)
        {
            robot.output.moveClampIntoRobot();
        }
        if (gamepad2.y)
        {
            robot.output.moveClampOutOfRobot();
        }

        if (gamepad2.right_bumper)
        {
            robot.output.startOpeningClamp();
        }
        else if (gamepad2.left_bumper)
        {
            robot.output.startClosingClamp();
        }
        else {
            robot.output.stopClamp();
        }

    }

    public void doFoundation()
    {
        if(gamepad2.dpad_left)
            robot.hooks.lowerHooks();

        if(gamepad2.dpad_right)
            robot.hooks.raiseHooks();
    }
}




