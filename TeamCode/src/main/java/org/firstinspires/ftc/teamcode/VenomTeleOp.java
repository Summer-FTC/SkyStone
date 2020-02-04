package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.io.PrintWriter;
import java.io.StringWriter;

@TeleOp(name = "VenomTeleOp", group = "6209")
public class VenomTeleOp extends OpMode
{
    VenomRobot robot = new VenomRobot();
    double drivePower = 1;
    boolean isReversed = false;
    boolean prevReversePressed = false;

    @Override
    public void init()
    {
        try {
            robot.init(hardwareMap, telemetry, false);
            robot.output.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.output.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.stoneHooks.startRaiseOneHook("L");
            robot.stoneHooks.startRaiseOneHook("R");

            telemetry.addData("Initialization complete", "");
            telemetry.update();
        } catch (Exception e) {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            telemetry.addData(sw.toString().substring(0, Math.min(500, sw.toString().length())), "");
            telemetry.update();
            try {
                Thread.sleep(10_000);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
                Thread.currentThread().interrupt();
            }
            throw new RuntimeException(e);
        }

    }

    public void loop()
    {
        doDrive();
        doFoundation();
        doOutput();
        doBrake();
        doStoneHooks();
    }



    void doDrive()
    {
        if(robot.driveTrain.isBraking()){
            return;
        }

        slowDown();

        double forward;
        double strafe;
        double rotate;

        if(gamepad1.back && !prevReversePressed){
            if(isReversed){
                forward = -gamepad1.left_stick_y * drivePower;
                strafe = -gamepad1.left_stick_x * drivePower;
                rotate = gamepad1.right_stick_x * drivePower;
            }
            else {
                forward = gamepad1.left_stick_y * drivePower;
                strafe = gamepad1.left_stick_x * drivePower;
                rotate = gamepad1.right_stick_x * drivePower;
            }
        } else {
            forward = -gamepad1.left_stick_y * drivePower;
            strafe = -gamepad1.left_stick_x * drivePower;
            rotate = gamepad1.right_stick_x * drivePower;
        }

        robot.driveTrain.arcadeDrive(forward, strafe, rotate);
    }

    public void doBrake(){

        if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right){
            robot.driveTrain.brake();
        }
        else {
            robot.driveTrain.unbrake();
        }
    }


    public void doStoneHooks()
    {
        if(gamepad2.right_trigger > 0.5){
            robot.stoneHooks.startLowerOneHook("R");
        } else {
            robot.stoneHooks.startRaiseOneHook("R");
        }
        if(gamepad2.left_trigger > 0.5){
            robot.stoneHooks.startLowerOneHook("L");
        } else {
            robot.stoneHooks.startRaiseOneHook("L");
        }
    }


    public void slowDown() {
        if (gamepad1.left_trigger > 0.5) {
            drivePower = 0.25;
        } else if (gamepad1.right_trigger > 0.5) {
            drivePower = 0.5;
        } else {
            drivePower = 1;
        }

        // Coast if and only if we are at full power.
        robot.driveTrain.setHaltModeCoast(drivePower == 1);
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

        if (gamepad2.b)
        {
            if (gamepad2.y)
            {
                robot.output.moveClampIntoRobot();
            }
        }
        else if (gamepad2.y)
        {
            robot.driveTrain.stopDriveMotors();

            if (Math.max(Math.max(Math.abs(gamepad2.left_stick_x), Math.abs(gamepad2.right_stick_x)),
                    Math.max(Math.abs(gamepad2.left_stick_y), Math.abs(gamepad2.right_stick_y))) > 0.5) {
                robot.output.moveClampOutOfRobotFromUnderHooks(robot);
            } else {

                // This is the normal one.
                robot.output.moveClampOutOfRobot();
            }
        }

        if (gamepad2.right_bumper)
        {
            robot.output.startOpeningClamp();
        }

        else if (gamepad2.left_bumper)
        {
            robot.output.startClosingClamp();
        }

        else
        {
            robot.output.stopClamp();
        }

        if(gamepad2.dpad_right){
            robot.output.setElbowPositions(1);
            robot.output.wrist.setPosition(0.97);
        }

    }

    public void doFoundation()
    {
        if(gamepad2.a||gamepad1.a)
        {
            robot.foundationHooks.lowerHooks();
        }
        else if(gamepad2.x || gamepad1.x)
        {
            robot.foundationHooks.raiseHooks();
        }
    }

//    }
}




