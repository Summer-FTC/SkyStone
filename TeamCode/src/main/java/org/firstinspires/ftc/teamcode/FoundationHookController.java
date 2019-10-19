package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationHookController
{
    public Servo servoHookL;
    public Servo servoHookR;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        servoHookL = hwMap.servo.get("servoHookL");
        servoHookR = hwMap.servo.get("servoHookR");

        telemetry.addData("Hook Servo Initialization Complete", "");
    }

    public void extendHooks() {
        // extend hooks
    }

    public void retractHooks() {
        // retract hooks
    }
}
