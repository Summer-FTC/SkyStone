package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationHookController
{
    public Servo leftHook;
    public Servo rightHook;

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

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


}
