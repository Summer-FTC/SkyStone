package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{

    public DcMotor motorLift;
    public Servo elbow1;
    public Servo elbow2;
    public Servo wrist;
    public CRServo clamp;

    HardwareMap hwMap;
    Telemetry telemetry;

    double liftPower = 1;


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");
        // elbows flip
        // 1 and 2 work simultaneously, might need to turn in opposite directions
        elbow1 = hwMap.servo.get("elbow1");
        elbow2 = hwMap.servo.get("elbow2");

        // wrist rotates block
        wrist = hwMap.servo.get("wrist");
        // clamp opens and closes on block
        clamp = hwMap.crservo.get("clamp");

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

    public void setPower(boolean extend) {
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

    public void extendLift()
    {
        // something with motorOutput
    }

    public void lowerLift()
    {
        // something with motorOutput
    }

    public void openClamp()
    {
        // something with servoClamp
    }

    public void closeClamp()
    {
        // something with servoClamp
    }

    // change the name of this
    public void dunk()
    {

    }
}