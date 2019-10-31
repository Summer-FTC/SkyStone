package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveController {
    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    String color;

    HardwareMap hwMap = null;
    Telemetry telemetry = null;

    public DcMotor[] motors = null;

    public MecanumDriveController() {
        // default to red i guess
        this.color = "red";
    }

    public MecanumDriveController(String color) {
        this.color = color;
    }

    public void init(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorFR = hwMap.dcMotor.get("motorFR");
        motorFL = hwMap.dcMotor.get("motorFL");
        motorBR = hwMap.dcMotor.get("motorBR");
        motorBL = hwMap.dcMotor.get("motorBL");

        motors = new DcMotor[]{motorFL, motorBL, motorBR, motorFR};

        resetEncoders();
        runWithoutEncoders();

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        stopDriveMotors();

        telemetry.addData("Motor Initialization Complete", "");
    }

    public void setPowers(double power) {
        for (DcMotor m : motors) {
            m.setPower(power);
        }
    }

    public void stopDriveMotors() {
        for (DcMotor m : motors) {
            m.setPower(0);
        }

    }


//        public void setHaltModeCoast(boolean coastModeOn = true){
//        DcMotor.ZeroPowerBehavior behavior;
//        if(coastModeOn)behavior = DcMotor.ZeroPowerBehavior.FLOAT;
//        else behavior = DcMotor.ZeroPowerBehavior.BRAKE;
//    }
//


    public void setHaltModeCoast(boolean coastModeOn) {
        DcMotor.ZeroPowerBehavior behavior;
        if (coastModeOn) {
            behavior = DcMotor.ZeroPowerBehavior.FLOAT;
        } else {
            behavior = DcMotor.ZeroPowerBehavior.BRAKE;
        }

        for (DcMotor m : motors) {
            m.setZeroPowerBehavior(behavior);
        }
    }


    public void arcadeDrive(double forward, double strafe, double rotate) {
        double FL = 0.0;
        double FR = 0.0;
        double BL = 0.0;
        double BR = 0.0;

        // change tolerance, maybe make stop method so that brakes as approaching zero speed
        if (((Math.abs(Math.hypot(strafe, forward))) > 0.1) ||
                Math.abs(Math.atan2(forward, strafe) - Math.PI / 4) > .1) {

            // r can be sqrt(2)/2
            double r = Math.hypot(strafe, forward);
            double theta = Math.atan2(forward, -strafe) - Math.PI / 4;
            double rightX = -rotate;

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
        } else {
            // setHaltModeCoast(false);
            stopDriveMotors();
        }

        motorFL.setPower(FL);
        motorBL.setPower(BL);
        motorFR.setPower(FR);
        motorBR.setPower(BR);
    }

    public void resetEncoders() {

        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void runWithoutEncoders() {

        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }


    public void runUsingEncoders() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void runToPosition() {
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public double getEncoderTicks() {
        double avg = 0.0;
        avg += motorFL.getCurrentPosition();
        avg += motorFR.getCurrentPosition();
        avg += motorBL.getCurrentPosition();
        avg += motorBR.getCurrentPosition();
        avg /= 4;

        return avg;
    }
}

