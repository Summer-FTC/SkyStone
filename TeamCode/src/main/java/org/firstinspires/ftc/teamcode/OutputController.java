package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{

    public DcMotor motorLift;
    public Servo elbow1;
    public Servo elbow2;
    public Servo wrist;
    public CRServo clamp;
    int position = 1;

    HardwareMap hwMap;
    Telemetry telemetry;

    double liftPower = 0.5;


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");

        // elbows flip
        // 1 and 2 should work simultaneously
        elbow1 = hwMap.servo.get("elbow1");
        elbow2 = hwMap.servo.get("elbow2");
        // wrist rotates block
        wrist = hwMap.servo.get("wrist");
        // clamp opens and closes on block
        clamp = hwMap.crservo.get("clamp");

        // this is kind of what David was saying:

        // flip up 30 degrees, using elbow
        // turn 90 degrees, using wrist
        // flip 150 degrees, using elbow
        // turn 90 degrees, using wrist
            // deposit
        // three positions: inside, outside (high/deposit), outside (low/intake)
        // use dpad to switch: outside low --> dpad up --> outside high --> dpad up --> inside
            // need to move foundation hooks to different button
            // small movement by elbow to get to outside low
            // outside --> inside: opposite of going out


        telemetry.addData("Output Servo Initialization Complete", "");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Output Motor Initialization Complete", "");
    }

    public void setLiftPower(boolean extend) {
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

    public void moveToPosition(int newPos)
    {
        if (newPos == 1) {
            // inside

            wrist.setPosition(0.5);
            moveServosSimultaneously(elbow1, elbow2, 0.7, 0.3);

            wrist.setPosition(0);

            moveServosSimultaneously(elbow1, elbow2, 0.3, 0);

            // need to work simultaneously? And turn while moving
            // move a little at a time, using loop
            // servo .setPosition() anywhere from 0-1, from lower to upper limit

            openClamp(); // for certain amount of time
            //sleep(2000);

            position = 1;

        } else if (newPos == 2) {
            // outside high

            // go outward from position 1
            if (position == 1) {
                closeClamp();
                sleep(1000);
                moveServosSimultaneously(elbow1, elbow2, 0, 0.3);

                wrist.setPosition(0.5); // 90 degrees

                moveServosSimultaneously(elbow1, elbow2, 0.3, 0.7);
                wrist.setPosition(0);
            }

            // raise from position 3
            else if (position == 3) {
                moveServosSimultaneously(elbow1, elbow2, 1, 0.7);
            }

            position = 2;

        } else if (newPos == 3) {
            // outside low
            // lower from position 2;
            moveServosSimultaneously(elbow1, elbow2, 0.7, 1);

            position = 3;

            openClamp();
            sleep(2000);

        }
    }

    public void openClamp()
    {
        // have to set power since continuous servo
        // I want to be able to do it for a certain amt of time, but haven't figured that out yet
        clamp.setPower(1);
        // idk what power
    }

    public void closeClamp()
    {
        clamp.setPower(-1);
    }


    public void moveServosSimultaneously(Servo servo1, Servo servo2, double start, double end) {
        // since we have two servos controlling the "elbow", I made this method to move them together
        // idk if it works

        for (double i = start + 0.05; i <= end; i+=0.05) {
            servo1.setPosition(i);
            servo2.setPosition(i);
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}