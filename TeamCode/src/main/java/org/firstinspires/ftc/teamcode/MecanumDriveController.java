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

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public DcMotor[] motors = null;


    public MecanumDriveController()
    {
        // default to red i guess
        this.color = "red";
    }


    public MecanumDriveController(String color)
    {
        this.color = color;
    }

    public void log(String message)
    {
        if (telemetry != null)
        {
            telemetry.addData(message, "");
            telemetry.update();
        }
    }


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        log("MecanumDriveController::init");

        motorFR = hwMap.dcMotor.get("motorFR");
        motorFL = hwMap.dcMotor.get("motorFL");
        motorBR = hwMap.dcMotor.get("motorBR");
        motorBL = hwMap.dcMotor.get("motorBL");

        motors = new DcMotor[]{motorFL, motorBL, motorBR, motorFR};

        resetEncoders();
        runWithoutEncoders();

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        // stopDriveMotors();

        log("Motor Initialization Complete");
    }

    public void setPowers(double power)
    {
        for (DcMotor m : motors) {
            m.setPower(power);
        }
    }


    public void stopDriveMotors()
    {
        for (DcMotor m : motors) {
            m.setPower(0);
        }
    }


    public void setHaltModeCoast(boolean coastModeOn)
    {
        DcMotor.ZeroPowerBehavior behavior;
        if (coastModeOn)
        {
            behavior = DcMotor.ZeroPowerBehavior.FLOAT;
        }
        else
         {
            behavior = DcMotor.ZeroPowerBehavior.BRAKE;
         }

        for (DcMotor m : motors)
        {
            m.setZeroPowerBehavior(behavior);
        }
    }


    public void arcadeDrive(double forward, double strafe, double rotate)
    {
        double FL = 0.0;
        double FR = 0.0;
        double BL = 0.0;
        double BR = 0.0;

        if (((Math.abs(Math.hypot(strafe, forward))) > 0.1) ||
                Math.abs(Math.atan2(forward, strafe) - Math.PI / 4) > .1)
        {
            double r = Math.hypot(strafe, forward);
            double theta = Math.atan2(forward, -strafe) - Math.PI / 4;
            double rightX = -rotate;

            FL = r * Math.cos(theta) + rightX;
            FR = r * Math.sin(theta) - rightX;
            BL = r * Math.sin(theta) + rightX;
            BR = r * Math.cos(theta) - rightX;


            if (((Math.abs(FL) > 1) || (Math.abs(BL) > 1)) || ((Math.abs(FR) > 1) || (Math.abs(BR) > 1))) {
                FL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                FR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
            }
        }

        else
         {
            // setHaltModeCoast(false);
            stopDriveMotors();
         }

        motorFL.setPower(FL);
        motorBL.setPower(BL);
        motorFR.setPower(FR);
        motorBR.setPower(BR);
    }


    public void resetEncoders()
    {

        for (DcMotor m : motors)
        {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    public void runWithoutEncoders()
    {

        for (DcMotor m : motors)
        {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void runUsingEncoders()
    {
        log("Run using encoders");

        for (DcMotor m : motors)
        {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runToPosition()
    {
        for (DcMotor m : motors)
        {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public double getAvgEncoderTicks()
    {
        double avg = 0.0;
        avg += motorFL.getCurrentPosition();
        avg += motorFR.getCurrentPosition();
        avg += motorBL.getCurrentPosition();
        avg += motorBR.getCurrentPosition();
        avg /= 4;

        return avg;
    }
}

