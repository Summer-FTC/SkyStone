package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "AutoTest" , group = "6209")
@Disabled
public class AutoTest extends BaseLinearOpMode
{
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        log("runOpMode", "running");
        try {
            test();
        } catch(Exception e){
            log("exception", e);
        }
    }

    public void test() {
        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();
        initialize();
        waitForStart();

        robot.output.moveClampOutOfRobot();
        sleep(2000);

        robot.output.moveClampIntoRobot();
        sleep(2000);

//        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);
//        sleep(2000);
//
//        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_INSIDE_ROBOT);
//        sleep(2000);
//
//        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);
//        sleep(2000);

//        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_INSIDE_ROBOT);
//        sleep(2000);
    }


    public void main() throws InterruptedException
    {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");

        runUsingEncoders();

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        log("Status", "Motor initialization complete");

        driveForwardDistance(0.5, 5000);
    }

    public void stopDriving()
    {
        driveForward(0);
    }

    public void driveForward(double power)
    {
        log("Status", "Driving forward with power: " + power);
        // all go forward
        motorBL.setPower(power);
        motorBR.setPower(power);
        motorFL.setPower(power);
        motorFR.setPower(power);
    }

    public void driveBackward(double power)
    {
        // all go backward
        motorBL.setPower(-power);
        motorBR.setPower(-power);
        motorFL.setPower(-power);
        motorFR.setPower(-power);
    }

    public void strafeLeft(double power)
    {
        // FL & BR go forward
        // FR & BL go backward
        motorBL.setPower(-power);
        motorBR.setPower(power);
        motorFL.setPower(power);
        motorFR.setPower(-power);
    }

    public void strafeRight(double power)
    {
        // FR & BL go forward
        // FL & BR go backward
        motorBL.setPower(power);
        motorBR.setPower(power);
        motorFL.setPower(power);
        motorFR.setPower(power);
    }

    @SuppressLint("DefaultLocale")
    public void driveForwardDistance(double power, int distance) throws InterruptedException
    {
        resetEncoders();

        // set target distance
        motorFR.setTargetPosition(distance);
        motorBR.setTargetPosition(distance);
        motorFL.setTargetPosition(distance);
        motorBL.setTargetPosition(distance);

        runToPosition();

        driveForward(power);

        log("Current Position", String.format("FR=%d BR=%d FL=%d BL=%d",
                motorFR.getCurrentPosition(),
                motorBR.getCurrentPosition(),
                motorFL.getCurrentPosition(),
                motorBL.getCurrentPosition()));

        log("Target Position", String.format("FR=%d BR=%d FL=%d BL=%d",
                motorFR.getTargetPosition(),
                motorBR.getTargetPosition(),
                motorFL.getTargetPosition(),
                motorBL.getTargetPosition()));

        while(motorFL.isBusy() || motorBL.isBusy())
        {
            log("Current Position", String.format("FR=%d BR=%d FL=%d BL=%d",
                    motorFR.getCurrentPosition(),
                    motorBR.getCurrentPosition(),
                    motorFL.getCurrentPosition(),
                    motorBL.getCurrentPosition()));

            log("Target Position", String.format("FR=%d BR=%d FL=%d BL=%d",
                    motorFR.getTargetPosition(),
                    motorBR.getTargetPosition(),
                    motorFL.getTargetPosition(),
                    motorBL.getTargetPosition()));


            Thread.sleep(100);
        }

        log("Status","done");
        stopDriving();
        resetEncoders();
    }

    public void resetEncoders()
    {
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition()
    {
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders()
    {
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void log(String caption, Object message)
    {
        telemetry.addData(caption, message);
        telemetry.update();
    }

}
