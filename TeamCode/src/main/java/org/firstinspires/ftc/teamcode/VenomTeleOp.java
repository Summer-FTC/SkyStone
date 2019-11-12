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
        // doOutake();
    }

    void doDrive()
    {
        double forward = -gamepad1.left_stick_y * drivePower;
        double strafe = -gamepad1.left_stick_x * drivePower;
        double rotate = gamepad1.right_stick_x * drivePower;


        robot.driveTrain.arcadeDrive(forward, strafe, rotate);
        robot.driveTrain.arcadeDrive(forward, strafe, rotate);

        slowDown(gamepad1.left_trigger, 0.25);
        slowDown(gamepad1.right_trigger, 0.5);
    }

    public void slowDown(float trigger, double speed) {

        if(trigger > 0.8 && drivePower == 1)
            drivePower = speed;
        else if (trigger > 0.8 && drivePower != 1)
            drivePower = 1;
    }

    public void doIntake()
    {
        if(gamepad2.right_bumper)
            robot.intake.setPower(intakePower);
        if (gamepad2.left_bumper)
            robot.intake.setPower(-intakePower);
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
            robot.hooks.lowerHooks();
            telemetry.addData("After lowering", "");
            telemetry.update();
        }

        if(gamepad2.dpad_up){
            robot.hooks.raiseHooks();
        }
    }
}
