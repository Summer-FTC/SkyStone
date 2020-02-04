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

    private boolean isBraking = false;

    private DcMotor.ZeroPowerBehavior currentZeroPowerBehavior = DcMotor.ZeroPowerBehavior.UNKNOWN;

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
            telemetry.addLine(message);
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

    public boolean isBraking(){
        return isBraking;
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
        if (coastModeOn) {
            behavior = DcMotor.ZeroPowerBehavior.FLOAT;
        } else {
            behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        }

        if (behavior != currentZeroPowerBehavior) {
            for (DcMotor m : motors) {
                m.setZeroPowerBehavior(behavior);
            }
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

        double maxAbs = Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BR), Math.abs(BL)));
        // Don't divide by 0.
        double scaledPower;
        if(maxAbs != 0){
            scaledPower = 1/maxAbs;
        }
        else{
            scaledPower = 0;
        }

        double hypLen = Math.sqrt(((forward*forward) + (strafe*strafe)));

        if(hypLen > 0.9){
            hypLen = 1;
        }

        scaledPower *= hypLen;

        FL *= scaledPower;
        BL *= scaledPower;
        FR *= scaledPower;
        BR *= scaledPower;


        telemetry.addData("Scaled Power", scaledPower);
        telemetry.addData("Hypotenuse length", hypLen);
        telemetry.addData("FL, BL, FR, BR", FL + ", " + BL +  ", " + FR +  ", "+ BR);
        telemetry.update();

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


    public void brake(){
        if(!isBraking) {
            setHaltModeCoast(false);
            // If the powers are already at 0, then it won't stop. So set a very small
            // power first so setting it 0 will do something.
            setPowers(0.01);
            stopDriveMotors();
            isBraking = true;
            log("Braked");
        }
    }

    public void unbrake(){
        if(isBraking) {
            setHaltModeCoast(true);
            isBraking = false;
            log("Unbraked");
        }
    }
}

