package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VenomRobot
{
    HardwareMap hwMap;
    public Telemetry telemetry = null;
    boolean isAuto = false;

    IMU imu ;

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

        telemetry.addData("Robot initialized", "");
        telemetry.update();

        driveTrain = new MecanumDriveController();
        driveTrain.init(hwMap, telemetry);

        imu = new IMU(hwMap.get(BNO055IMU.class, "imu"));
        imu.IMUinit(hwMap);
//
//        intake = new IntakeController();
//        intake.init(hwMap, telemetry);
//
//        output = new OutputController();
//        output.init(hwMap,telemetry);
//
//        hooks = new FoundationHookController();
//        hooks.init(hwMap,telemetry);
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

    // TODO: FIX
    public double getEncoderTicks()
    {
        double avg = 0.0;
        avg += driveTrain.motorFL.getCurrentPosition();
        avg += driveTrain.motorFR.getCurrentPosition();
        avg += driveTrain.motorBL.getCurrentPosition();
        avg += driveTrain.motorBR.getCurrentPosition();
        avg /= 4;

        return avg;
    }

}
