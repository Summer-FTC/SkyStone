package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VenomHardwareMap {
    HardwareMap hwMap = null;
    public Telemetry telemetry = null;
    LinearOpMode opMode = null;

    BNO055IMU imu = null;
    public DcMotor mtrFL;
    public DcMotor mtrFR;
    public DcMotor mtrBL;
    public DcMotor mtrBR;
    public Servo leftFoundation;
    public Servo rightFoundation;
    public Servo ;
    public Servo ;
    public Servo ;
    public Servo ;

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous){
        this.hwMap = hwMap;
        this.telemetry = telemetry;


    }

}
