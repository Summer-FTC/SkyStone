package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveController
{
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    String color;

    public MecanumDriveController()
    {
        // default to red i guess
        this.color = "red";
    }
    public MecanumDriveController(String color)
    {
       this.color = color;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        motorFR = hwMap.dcMotor.get("motorFR");
        motorFL = hwMap.dcMotor.get("motorFL");
        motorBR = hwMap.dcMotor.get("motorBR");
        motorBL = hwMap.dcMotor.get("motorBL");

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        stopDriveMotors();

        telemetry.addData("Motor Initialization Complete", "");



    }

    public void moveForward()
    {

    }

    public void moveBackward()
    {

    }

    public void strafeRight()
    {

    }

    public void strafeLeft()
    {

    }

    public void moveToLoadingZone()
    {

    }

    public void rotate()
    {

    }

    public void stopDriveMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }

    public void arcadeDrive(double forward, double strafe, double rotate)
    {
        double FL = 0.0;
        double FR = 0.0;
        double BL = 0.0;
        double BR = 0.0;

        if (((Math.abs(Math.hypot(strafe, forward))) > .1) ||
                Math.abs(Math.atan2(forward, strafe) - Math.PI / 4) > .1) {

            // r can be sqrt(2)/2
            double r = Math.hypot(strafe, forward);
            double theta = Math.atan2(forward, -strafe) - Math.PI / 4;
            double rightX = -rotate;

            // as per unit circle cos gives x, sin gives you y
            FL = r * Math.cos(theta) + rightX;
            FR = r * Math.sin(theta) - rightX;
            BL = r * Math.sin(theta) + rightX;
            BR = r * Math.cos(theta) - rightX;

            // Don't give a value greater than 1 so scale them all down
            // would we ever get a value greater than 1? root2?
            // this might be Jank, fix later
            if (((Math.abs(FL) > 1) || (Math.abs(BL) > 1)) || ((Math.abs(FR) > 1) || (Math.abs(BR) > 1))) {
                FL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                FR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
            }
        }

        // is this needed? it would be 0.0 anyways
        else
        {
            stopDriveMotors();
        }

        motorFL.setPower(FL);
        motorBL.setPower(BL);
        motorFR.setPower(FR);
        motorBR.setPower(BR);
    }
}
