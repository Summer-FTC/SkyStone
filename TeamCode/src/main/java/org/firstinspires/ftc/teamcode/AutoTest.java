package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;


@Autonomous
public class AutoTest extends CustomLinearOpMode
{
    DcMotor motorFR = null;
    DcMotor motorFL = null;
    DcMotor motorBR = null;
    DcMotor motorBL = null;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("runOpMode()", "");
        main();
    }


    public void main() throws InterruptedException
    {
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");

        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Motor Initialization Complete", "");

        driveForwardDistance(0.5, 100);

    }

    public void stopDriving()
    {
        driveForward(0);
    }

    public void driveForward(double power)
    {
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

    public void driveForwardDistance(double power, int distance) //throws InterruptedException
    {
        resetEncoders();

        // set target distance
        motorFR.setTargetPosition(distance);
        motorBR.setTargetPosition(distance);
        motorFL.setTargetPosition(distance);
        motorBL.setTargetPosition(distance);

        runToPosition();

        driveForward(power);


        while(motorFL.isBusy() && motorBL.isBusy())
        {
            telemetry.addData("busy...", "");

            Thread.yield();
        }

        telemetry.addData("done", "");
        stopDriving();
        runUsingEncoders();


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

}
