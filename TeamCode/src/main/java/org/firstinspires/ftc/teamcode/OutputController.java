package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{

    public DcMotor motorLift;
    public Servo leftClamp;
    public Servo rightClamp;

    HardwareMap hwMap;
    Telemetry telemetry;

    double liftPower = 1;


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");
        leftClamp = hwMap.servo.get("leftClamp");
        rightClamp = hwMap.servo.get("rightClamp");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Output Motor Initialization Complete", "");

        telemetry.addData("Output Servo Initialization Complete", "");
    }

    public void setPower(boolean extend) {
        if (extend) {
            motorLift.setPower(liftPower);
        } else {
            motorLift.setPower(-liftPower);
        }
    }

    public void stopOutputMotors()
    {
        motorLift.setPower(0);
    }

    public void extendLift()
    {
        // something with motorOutput
    }

    public void lowerLift()
    {
        // something with motorOutput
    }

    public void openClamp()
    {
        // something with servoClamp
    }

    public void closeClamp()
    {
        // something with servoClamp
    }

    // change the name of this
    public void dunk()
    {

    }
}