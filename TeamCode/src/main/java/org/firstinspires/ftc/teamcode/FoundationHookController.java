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

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        leftHook = hwMap.servo.get("leftHook");
        rightHook = hwMap.servo.get("rightHook");

        telemetry.addData("Hook Servo Initialization Complete", "");
    }


    public void raiseHooks()
    {
        leftHook.setPosition(1);
        rightHook.setPosition(0);

        // Wait for the hooks to raise.
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }

    public void lowerHooks()
    {
        leftHook.setPosition(0);
        rightHook.setPosition(1);

        // Wait for the hooks to lower.
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }

}
