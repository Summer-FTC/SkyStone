package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VenomTeleOp", group = "6209")
public class VenomTeleOp extends OpMode
{
    VenomRobot robot = new VenomRobot();
    double intakePower = 1;

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
        doIntake();
        doOutake();
        doFoundation();

        // add other methods such as DoLift or DoIntake
    }

    void doDrive()
    {


        double forward = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;


        robot.driveTrain.arcadeDrive(forward, strafe, rotate);
        robot.driveTrain.arcadeDrive(forward, strafe, rotate);




    }

    public void doIntake()
    {
        if(gamepad2.right_bumper)
        {
            robot.intake.setPower(intakePower);
        } if (gamepad2.left_bumper){
            robot.intake.setPower(-intakePower);
        }
    }

    public void doOutake(){
        if(gamepad2.right_trigger > 0.2){
            robot.output.setPower(true);
        }

        if(gamepad2.left_trigger > 0.2){
            robot.output.setPower(false);
        }
    }

    public void doFoundation(){
        if(gamepad2.dpad_down){
            robot.hooks.leftHook.setPosition(1);
            robot.hooks.rightHook.setPosition(1);
        }
        if(gamepad2.dpad_up){
            robot.hooks.leftHook.setPosition(0);
            robot.hooks.rightHook.setPosition(0);
        }
    }
}
