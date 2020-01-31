package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class CustomOpMode extends OpMode
{
   DcMotor motorFR;
   DcMotor motorFL;
   DcMotor motorBR;
   DcMotor motorBL;
 //   public DcMotor motorIntakeL;
   // public DcMotor motorIntakeR;
   // public DcMotor motorLift;
  //  public DcMotor motorOutput;

    // Declare servos
 //   public CRServo servoClamp;
 //   public CRServo servoTwist;
 //   public Servo servoHookL;
 //   public Servo servoHookR;

   IMU imu;

  // ModernRoboticsI2cRangeSensor rangeSensor;

   public void init()
   {
        initialize();
        loop();
   }

   public void loop()
   {
       arcadeDrive();
   }

   public void initialize()
   {
       motorFR = hardwareMap.dcMotor.get("motorFR");
       motorFL = hardwareMap.dcMotor.get("motorFL");
       motorBR = hardwareMap.dcMotor.get("motorBR");
       motorBL = hardwareMap.dcMotor.get("motorBL");
 //      motorIntakeL = hardwareMap.dcMotor.get("motorIntakeL");
 //      motorIntakeR = hardwareMap.dcMotor.get("motorIntakeR");
 //      motorLift = hardwareMap.dcMotor.get("motorLift");
 //      motorOutput = hardwareMap.dcMotor.get("motorOutput");

 //      rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensor");

       motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 //      motorIntakeL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 //      motorIntakeR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 //      motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       //      motorOutput.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //
       motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

       motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
       motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
       // motorIntakeL.setDirection(DcMotorSimple.Direction.REVERSE);

       stopDriveMotors();

       telemetry.addData("Motor Initialization Complete", "");

       // servoClamp = hardwareMap.crservo.get("servoClamp");
       // servoTwist = hardwareMap.crservo.get("servoTwist");
       // servoHookL = hardwareMap.servo.get("servoHookL");
       // servoHookR = hardwareMap.servo.get("servoHookR");

       // Set servo positions

       telemetry.addData("Servo Initialization Complete", "");

       imu = new IMU(hardwareMap.get(BNO055IMU.class, "imu"));
       imu.IMUinit(hardwareMap, "imu");

       telemetry.addData("IMU Initialization Complete", "");


       telemetry.addData("Initialization Complete", "");

       telemetry.addData("Initialization Complete", "");

   }

    // don't need mecanum drive class anymore ??
    public void arcadeDrive()
    {
        double FL = 0.0;
        double FR = 0.0;
        double BL = 0.0;
        double BR = 0.0;

        if (((Math.abs(Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y))) > .1) ||
                Math.abs(Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4) > .1) {

            // r can be sqrt(2)/2
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            double theta = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;

            // as per unit circle cos gives x, sin gives you y
            FL = r * Math.cos(theta) + rightX;
            FR = r * Math.sin(theta) - rightX;
            BL = r * Math.sin(theta) + rightX;
            BR = r * Math.cos(theta) - rightX;

            // Don't give a value greater than 1 so scale them all down
            // would we ever get a value greater than 1? root2?
            // this might be Jank, fix later
            if (((Math.abs(FL) > 1) || (Math.abs(BL) > 1)) || ((Math.abs(FR) > 1) || (Math.abs(BR) > 1))) {
                FL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BL /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                FR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
                BR /= Math.max(Math.max(Math.abs(FL), Math.abs(FR)), Math.max(Math.abs(BL), Math.abs(BR)));
            }
        }

        // is this needed? it would be 0.0 anyways
        else
        {
            stopDriveMotors();
        }

        motorFL.setPower(FL);
        motorBL.setPower(BL);
        motorFR.setPower(FR);
        motorBR.setPower(BR);
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

    // Idk if we need these lol just copied them from last year
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

    public double getDist()
    {
        double dist = 0.0;
        /*
                rangeSensor.getDistance(DistanceUnit.INCH);
        while ((dist > 200 || Double.isNaN(dist))) {
            dist = rangeSensor.getDistance(DistanceUnit.INCH);
        }

         */
        return dist;
    }
}