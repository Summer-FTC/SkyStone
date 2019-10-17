package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VenomRobot
{
    HardwareMap hwMap;
    public Telemetry telemetry = null;
    boolean isAuto = false;

    MecanumDriveController drive = null;
    OutputController output = null;

    public VenomRobot(MecanumDriveController drive)
    {
        this.drive = drive;
    }


    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAuto)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isAuto = isAuto;


        drive.init(hwMap, telemetry);
    }


}
