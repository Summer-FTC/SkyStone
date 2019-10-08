package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CustomOpMode extends OpMode
{
   DcMotor motorFR;
   DcMotor motorFL;
   DcMotor motorBR;
   DcMotor motorBL;
    public DcMotor motorIntakeL;
    public DcMotor motorIntakeR;
    public DcMotor motorLift;
    public DcMotor motorOutput;

    // Declare servos
    public CRServo servoClamp;
    public CRServo servoTwist;
    public Servo servoHookL;
    public Servo servoHookR;

   IMU imu;

   ModernRoboticsI2cRangeSensor rangeSensor;

   public void init()
   {
        initialize();
        loop();
   }

   public void loop()
   {
       // extend mecanum drive class?
       /*
            // Vd = Desired robot speed [−1,1]
            // angle = Desired robot angle [0, 360]
            // Vt = Desired speed for changing direction [−1,1]

            // double angle = gamepad.leftStick(Get360degrees)
            // double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            // int Vd = gamepad.
            // int

           motorBR.setPower(WheelSpeed(Vd, angle, Vt));

        */

   }

   public void initialize()
   {
       motorFR = hardwareMap.dcMotor.get("motorFR");
       motorFL = hardwareMap.dcMotor.get("motorFL");
       motorBR = hardwareMap.dcMotor.get("motorBR");
       motorBL = hardwareMap.dcMotor.get("motorBL");
       motorIntakeL = hardwareMap.dcMotor.get("motorIntakeL");
       motorIntakeR = hardwareMap.dcMotor.get("motorIntakeR");
       motorLift = hardwareMap.dcMotor.get("motorLift");
       motorOutput = hardwareMap.dcMotor.get("motorOutput");

       rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

       motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorIntakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorIntakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorOutput.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
       motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
       motorIntakeL.setDirection(DcMotorSimple.Direction.REVERSE);

       stopDriveMotors();

       telemetry.addData("Motor Initialization Complete", "");

       servoClamp = hardwareMap.crservo.get("servoClamp");
       servoTwist = hardwareMap.crservo.get("servoTwist");
       servoHookL = hardwareMap.servo.get("servoHookL");
       servoHookR = hardwareMap.servo.get("servoHookR");

       // set servo positions

       telemetry.addData("Servo Initialization Complete", "");

       imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
       imu.IMUinit(hardwareMap);

       telemetry.addData("IMU Initialization Complete", "");

       telemetry.addData("Initialization Complete", "");
   }

    public void stopDriveMotors() {
        motorFR.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorBL.setPower(0);
    }


    public void setLeftMotors(double left){
        motorFL.setPower(left);
        motorBL.setPower(left);
    }

    public void setRightMotors(double right){
        motorFR.setPower(right);
        motorBR.setPower(right);
    }


    public double rightABSMotorVal(double joyStickVal) {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorBR.getPower() + maxJump) {
            return Range.clip(motorBR.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBR.getPower() - maxJump) {
            return Range.clip(motorBR.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }


    public double leftABSMotorVal(double joyStickVal) {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorBL.getPower() + maxJump) {
            return Range.clip(motorBL.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorBL.getPower() - maxJump) {
            return Range.clip(motorBL.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }

    public double getDist() {
        double dist = rangeSensor.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist))) {
            dist = rangeSensor.getDistance(DistanceUnit.INCH);
        }
        return dist;
    }


}