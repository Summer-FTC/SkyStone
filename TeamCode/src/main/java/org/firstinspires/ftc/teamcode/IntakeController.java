package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeController
{
    public DcMotor motorIntakeL;
    public DcMotor motorIntakeR;

    HardwareMap hwMap;
    Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorIntakeL = hwMap.dcMotor.get("motorIntakeL");
        motorIntakeR = hwMap.dcMotor.get("motorIntakeR");
    }

    public void setPower(double power)
    {
        motorIntakeL.setPower(power);
        motorIntakeR.setPower(power);
    }
}
