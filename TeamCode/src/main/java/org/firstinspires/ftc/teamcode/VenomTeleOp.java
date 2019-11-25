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
        // doOutput();
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

    public void doIntake()
    {
        if(gamepad2.right_bumper)
            robot.intake.setPower(intakePower);
        if (gamepad2.left_bumper)
            robot.intake.setPower(-intakePower);
    }

    public void doOutput(){
        if(gamepad2.right_trigger > 0.2){
            robot.output.setPower(true);
        }

        if(gamepad2.left_trigger > 0.2){
            robot.output.setPower(false);
        }


    }

    public void doFoundation(){
        if(gamepad2.left_bumper)
            robot.hooks.lowerHooks();

        if(gamepad2.right_bumper)
            robot.hooks.raiseHooks();
    }
}
