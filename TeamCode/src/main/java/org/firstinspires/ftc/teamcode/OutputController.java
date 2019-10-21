package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    }

    public void setPower(boolean extend){
        if(extend){
            motorLift.setPower(liftPower);
        } else {
            motorLift.setPower(-liftPower);
        }

    }

    public void openClamp()
    {

    }

    public void closeClamp()
    {

    }

    // change the name of this
    public void dunk()
    {

    }
}