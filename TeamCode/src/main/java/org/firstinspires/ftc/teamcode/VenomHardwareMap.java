package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

<<<<<<< HEAD
public class VenomHardwareMap {
    public DcMotor mtrBR;
    public DcMotor mtrBL;
=======
public class VenomHardwareMap
{
    HardwareMap hwMap = null;
    public Telemetry telemetry = null;
    LinearOpMode opMode = null;

    BNO055IMU imu = null;
>>>>>>> adc9fb61bfb99ac284e8f05feee942fa3ce35e5f
    public DcMotor mtrFL;
    public DcMotor mtrFR;
    public Servo leftFoundation;
    public Servo rightFoundation;
    public Servo servo1;
    public Servo servo2;
    public Servo servo3;
    public Servo servo4;
    public Servo capStone;
    public Servo numberTwo;
    public Servo numberThree;
    public Servo numberFour;
    boolean isAutonomous = false;
    public
    BNO055IMU imu = null;


    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAutonomous) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.isAutonomous = isAutonomous;

        mtrBL.setDirection

        capStone = hwMap.get(Servo.class, "capStone");

        if(isAutonomous) initIMU();


    }


    public static class Motion(){
        public final double vD;
        public final double thetaD;
        public final double vTheta;
    }

    public Motion (double vD, double thetaD ,double vTheta){
        this vD = vD;
        this thetaD = thetaD;
        this vTheta = vTheta;
    }

    public static void driveFRS (double forward, double rotate, double sideways, double maxPower){
    Motion motion = createMotionFromFRS(sideways, forward, rotate);



    }
    public static Motion createMotionFromFRS(double forward, double sideways, double rotate) {
        // V_d = desired robot speed.
        // theta_d = desired robot velocity angle.
        // V_theta = desired robot rotational speed.
        double vD = Math.min(Math.sqrt(Math.pow(sideways, 2) + Math.pow(forward, 2)), 1);
        double thetaD = Math.atan2(-sideways, -forward);
        double vTheta = -rotate;
        return new Motion(vD, thetaD, vTheta);
    }

}
