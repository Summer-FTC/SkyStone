package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeController
{
    public DcMotor motorIntake = null;
    public Servo leftClamp = null;
    public Servo rightClamp = null;

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorIntake = hwMap.dcMotor.get("motorIntake");
        leftClamp = hwMap.servo.get("leftClamp");
        rightClamp = hwMap.servo.get("rightClamp");
    }

    public void setPower(double power)
    {
        motorIntake.setPower(power);
    }

    public void clampClose()
    {
        leftClamp.setPosition(1);
        rightClamp.setPosition(1);
    }

    public void clampOpen()
    {
        leftClamp.setPosition(0);
        rightClamp.setPosition(0);
    }
}
