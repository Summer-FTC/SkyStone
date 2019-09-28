package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TrollBotHardwareMap
{
    HardwareMap hwMap = null;
    public Telemetry telemetry = null;
    LinearOpMode opMode = null;
  //  private ElapsedTime period = newElapsedTime();

    BNO055IMU imu = null;
    public DcMotor mtrL;
    public DcMotor mtrR;
  //  public Servo leftFoundation;
  //  public Servo rightFoundation;
    //public Servo ;
    //public Servo ;
    //public Servo ;
    //public Servo ;

    public void init (HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous){
        this.hwMap = hwMap;

        this.telemetry = telemetry;
        mtrL = hwMap.dcMotor.get("left_drive");
        mtrR = hwMap.dcMotor.get("right_drive");
        mtrL.setDirection(DcMotor.Direction.REVERSE);

        mtrL.setPower(0);
        mtrR.setPower(0);

        mtrL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

}
