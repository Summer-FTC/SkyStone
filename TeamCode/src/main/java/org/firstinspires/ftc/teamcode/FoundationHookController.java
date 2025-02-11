package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FoundationHookController
{
    public Servo leftHook;
    public Servo rightHook;

    HardwareMap hwMap;
    Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        rightHook = hwMap.servo.get("rightHook");
        leftHook = hwMap.servo.get("leftHook");

        startRaiseHooks();

      //  telemetry.addData("Hook Servo Initialization Complete", "");
    }


    public void startRaiseHooks()
    {
        leftHook.setPosition(1);
        rightHook.setPosition(0);
    }

    public void startLowerHooks()
    {
        leftHook.setPosition(0.21);
        rightHook.setPosition(1);
    }

    public void raiseHooks()
    {
        startRaiseHooks();

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
        leftHook.setPosition(0.21);
        rightHook.setPosition(1);

        // Wait for the hooks to lower.
        try {
            Thread.sleep(1500);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }

    public boolean areHooksDown()
    {
        if (leftHook.getPosition() < 0.4 && rightHook.getPosition()>0.8)
        {
            return true;
        }
        return false;
    }

}
