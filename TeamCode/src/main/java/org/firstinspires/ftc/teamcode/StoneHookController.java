package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class StoneHookController
{
    private static final int LOWER_TIME_MILLIS = 1000;

    // We don't have to wait as long for them to raise before we can move.
    private static final int RAISE_TIME_MILLIS = 500;

    public Servo leftStoneHook;
    public Servo rightStoneHook;

    HardwareMap hwMap;
    Telemetry telemetry;

    public void init(HardwareMap hardwareMap, Telemetry telemetry){
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        rightStoneHook = hwMap.servo.get("rightStoneHook");
        leftStoneHook = hwMap.servo.get("leftStoneHook");

        startRaiseHooks();

        telemetry.addData("Hook Servo Initialization Complete", "");
    }


    public void startRaiseHooks()
    {
        leftStoneHook.setPosition(0);
        rightStoneHook.setPosition(0);
    }

    public void raiseOneHook(String LorR)
    {
        if(LorR.equals("R"))
        {
            rightStoneHook.setPosition(0);
        }

        else{
            leftStoneHook.setPosition(0);
        }

        // Wait for the hooks to raise.
        try {
            Thread.sleep(RAISE_TIME_MILLIS);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }

    public void lowerOneHook(String LorR)
    {
        if(LorR.equals("R"))
        {
            rightStoneHook.setPosition(1);
        }
        else{
            leftStoneHook.setPosition(1);
        }

        // Wait for the hooks to raise.
        try {
            Thread.sleep(LOWER_TIME_MILLIS);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }




    public void raiseHooks()
    {
        startRaiseHooks();

        // Wait for the hooks to raise.
        try {
            Thread.sleep(RAISE_TIME_MILLIS);
        } catch (InterruptedException e) {
            // This is important to propagate the interrupt up.
            Thread.currentThread().interrupt();
        }
    }
}
