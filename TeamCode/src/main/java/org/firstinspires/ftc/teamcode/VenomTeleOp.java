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

    boolean allowDoorChanges = true;


    @Override
    public void init()
    {
        try {
            robot.init(hardwareMap, telemetry, false);
            robot.output.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.output.motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.stoneHooks.startRaiseOneHook("L");
            robot.stoneHooks.startRaiseOneHook("R");

            robot.output.cap.setPosition(1);

          //  telemetry.addData("Initialization complete", "");
          //  telemetry.update();
        } catch (Exception e) {
//            StringWriter sw = new StringWriter();
//            PrintWriter pw = new PrintWriter(sw);
//            e.printStackTrace(pw);
//            telemetry.addData(sw.toString().substring(0, Math.min(500, sw.toString().length())), "");
//            telemetry.update();
//            try {
//                Thread.sleep(10_000);
//            } catch (InterruptedException ex) {
//                ex.printStackTrace();
//                Thread.currentThread().interrupt();
//            }
            throw new RuntimeException(e);
        }

    }

    long maxDurationMillis = 0;
    long totalLoops = 0;
    long totalLoopDurationMills = 0;
    public void loop()
    {
        long startMillis = System.currentTimeMillis();

        doDrive();
        doFoundation();
        doOutput();
        doBrake();
        doStoneHooks();
        doCapStoneDrop();

        long durationMillis = System.currentTimeMillis() - startMillis;
        totalLoopDurationMills += durationMillis;
        totalLoops++;

        maxDurationMillis = Math.max(durationMillis, maxDurationMillis);
        telemetry.addData("Loop duration (ms)", durationMillis);
        telemetry.addData("Loop duration avg (ms)", totalLoopDurationMills/totalLoops);
        telemetry.addData("Loop duration max (ms)", maxDurationMillis);
        telemetry.addData("Total Loops", totalLoops);
        telemetry.update();
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

        if(gamepad1.back && !isReversed){
            isReversed = true;
        }
        else if(gamepad1.start && isReversed){
            isReversed = false;
        }

        if(!isReversed){
            forward = -gamepad1.left_stick_y * drivePower;
            strafe = -gamepad1.left_stick_x * drivePower;
        }

        else{
            forward = gamepad1.left_stick_y * drivePower;
            strafe = gamepad1.left_stick_x * drivePower;
        }

        rotate = gamepad1.right_stick_x * drivePower;

        double hypLenR = Math.abs(rotate);
        robot.driveTrain.arcadeDrive(forward, strafe, rotate, hypLenR);
    }


    public void doBrake(){

        if(gamepad1.dpad_down || gamepad1.dpad_up){
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

//        if (gamepad2.b)
//        {
//            if (gamepad2.y)
//            {
//                robot.output.moveClampIntoRobot();
//            }
//        }
//        else if (gamepad2.y)
//        {
//            robot.driveTrain.stopDriveMotors();
//
//            if (Math.max(Math.max(Math.abs(gamepad2.left_stick_x), Math.abs(gamepad2.right_stick_x)),
//                    Math.max(Math.abs(gamepad2.left_stick_y), Math.abs(gamepad2.right_stick_y))) > 0.5) {
//                robot.output.moveClampOutOfRobotFromUnderHooks(robot);
//            } else {
//
//                // This is the normal one.
//                robot.output.moveClampOutOfRobot();
//            }
//        }
//
//        if (gamepad2.right_bumper)
//        {
//            robot.output.startOpeningClamp();
//        }
//
//        else if (gamepad2.left_bumper)
//        {
//            robot.output.startClosingClamp();
//        }
//
//        else
//        {
//            robot.output.stopClamp();
//        }
//
//        if(gamepad2.dpad_right){iu
//            robot.output.setElbowPositions(1);
//            robot.output.wrist.setPosition(0.97);
//        }

        if(gamepad2.y && gamepad2.b){
            robot.output.openDoorsFully();
            // We can't let go of y and b at exactly the same time, so ignore door changes
            // untl they are both released.
            allowDoorChanges = false;
        }

        else if(gamepad2.y){
            if (allowDoorChanges) {
                robot.output.openDoorsPartyway();
            }
        }

        else if(gamepad2.b){
            if (allowDoorChanges) {
                robot.output.shutDoors();
            }
        }
        else {
            // Both buttons have been released, so allow other changes.
            allowDoorChanges = true;
        }

        if(gamepad2.right_bumper) {
            robot.output.releaseCap();
        }

        else if(gamepad2.left_bumper){
            robot.output.pullCapBackIn();
        }

      //   if(gamepad2.back){
             // deployCapstone();
     //    }

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

    public void doCapStoneDrop(){
        if(gamepad1.dpad_left){
            robot.output.lowerCap();
        }

        else if (gamepad1.dpad_right){
            robot.output.bringCapIn();
        }
    }

}




