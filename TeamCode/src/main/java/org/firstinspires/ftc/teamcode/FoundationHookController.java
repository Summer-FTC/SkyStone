package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationHookController
{
<<<<<<< HEAD
    public Servo leftHook;
    public Servo rightHook;
=======
    public Servo servoHookL;
    public Servo servoHookR;
>>>>>>> 060a4a0e9df7ede0df64cd39d437b0c22c88485d

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

<<<<<<< HEAD
    public void init(HardwareMap hardwareMap,Telemetry telemetry){
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        leftHook = hwMap.servo.get("leftHook");
        rightHook = hwMap.servo.get("rightHook");
    }

    public void pullFoundation(boolean lower){
        if(lower){
            leftHook.setPosition(1);
            rightHook.setPosition(1);
        } else {
            leftHook.setPosition(0);
            rightHook.setPosition(0);
        }
    }

=======
    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        servoHookL = hwMap.servo.get("servoHookL");
        servoHookR = hwMap.servo.get("servoHookR");

        telemetry.addData("Hook Servo Initialization Complete", "");
    }

    public void extendHooks() {
        // extend hooks
    }
>>>>>>> 060a4a0e9df7ede0df64cd39d437b0c22c88485d

    public void retractHooks() {
        // retract hooks
    }
}
