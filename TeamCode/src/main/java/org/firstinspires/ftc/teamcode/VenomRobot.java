package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VenomRobot
{
    HardwareMap hwMap;
    public Telemetry telemetry = null;
    boolean isAuto = false;

    IMU imu;
    MecanumDriveController driveTrain;
    OutputController output;
    FoundationHookController hooks;
    IntakeController intake;


    public VenomRobot()
    {

    }


    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAuto)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isAuto = isAuto;

        log("VenomRobot::init");

        driveTrain = new MecanumDriveController();
        driveTrain.init(hwMap, telemetry);

        hooks = new FoundationHookController();
        hooks.init(hwMap, telemetry);

        imu = new IMU(hwMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hwMap);

        // intake = new IntakeController();
        // intake.init(hwMap, telemetry);
        //
        // output = new OutputController();
        // output.init(hwMap,telemetry);
  }


    public void setMotorFL(double power)
    {
        driveTrain.motorFL.setPower(power);
    }

    public void setMotorFR(double power)
    {
        driveTrain.motorFR.setPower(power);
    }

    public void setMotorBL(double power)
    {
        driveTrain.motorBL.setPower(power);
    }

    public void setMotorBR(double power)
    {
        driveTrain.motorBR.setPower(power);
    }

    public void log(String message)
    {
        if (telemetry != null)
        {
            telemetry.addData(message, "");
            telemetry.update();
        }
    }
}
