package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeController
{
    public DcMotor motorIntake;
    public Servo leftClamp;
    public Servo rightClamp;

    HardwareMap hwMap;
    Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        //motorIntake = hwMap.dcMotor.get("motorIntake");
        leftClamp = hwMap.servo.get("leftClamp");
        rightClamp = hwMap.servo.get("rightClamp");
    }

    public void setPower(double power)
    {
        motorIntake.setPower(power);
    }

    public void clampPositions(boolean clamp)
    {
        if(clamp){
            leftClamp.setPosition(1);
            rightClamp.setPosition(1);
        } else {
            leftClamp.setPosition(0);
            rightClamp.setPosition(0);
        }

    }


}
