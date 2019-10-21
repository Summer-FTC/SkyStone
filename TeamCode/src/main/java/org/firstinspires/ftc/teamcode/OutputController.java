package org.firstinspires.ftc.teamcode;

<<<<<<< HEAD
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
=======
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
>>>>>>> 060a4a0e9df7ede0df64cd39d437b0c22c88485d

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{
<<<<<<< HEAD

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

=======
    public DcMotor motorLift;
    public DcMotor motorOutput;

    public Servo servoTwist;
    public Servo servoClamp;

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");
        motorOutput = hwMap.dcMotor.get("motorOutput");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOutput.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorOutput.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopOutputMotors();

        telemetry.addData("Output Motor Initialization Complete", "");

        servoClamp = hwMap.servo.get("servoClamp");
        servoTwist = hwMap.servo.get("servoTwist");

        telemetry.addData("Output Servo Initialization Complete", "");
    }

    public void stopOutputMotors()
    {
        motorLift.setPower(0);
        motorOutput.setPower(0);
    }

    public void extendLift()
    {
        // something with motorOutput
    }

    public void lowerLift()
    {
        // something with motorOutput
>>>>>>> 060a4a0e9df7ede0df64cd39d437b0c22c88485d
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