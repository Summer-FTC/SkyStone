package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{
    // Positions for the wrist servo.
    private static final double WRIST_POSITION_SIDEWAYS = 0.45;
    private static final double WRIST_POSITION_INSIDE_ROBOT = 0;
    private static final double WRIST_POSITION_OUTSIDE_ROBOT= 1;

    // Positions for the elbow servo.
    private static final double ELBOW_POSITION_INSIDE_ROBOT = 0;
    private static final double ELBOW_POSITION_OUTSIDE_ROBOT_PARALLEL = 0.8;
    private static final double ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN = 1;

    private static final double LIFT_POWER_UP = 0.75;

    // This is negative to move down and has a much smaller absolute value since gravity
    // helps us down.
    private static final double LIFT_POWER_DOWN = -0.2;

    // How long to move the lift up and then down when moving the clamp in or out of the robot.
    private static final long MOVE_CLAMP_LIFT_DURATION = 1000;


    public DcMotor motorLift;
    public Servo elbowR;
    public Servo elbowL;
    public Servo wrist;
    public CRServo clamp;
    int position = 1;

    HardwareMap hwMap;
    Telemetry telemetry;


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");

        // elbows flip
        // 1 and 2 should work simultaneously
        elbowR = hwMap.servo.get("elbow1");
        elbowL = hwMap.servo.get("elbow2");
        // wrist rotates block
        wrist = hwMap.servo.get("wrist");
        // clamp opens and closes on block
        clamp = hwMap.crservo.get("clamp");

        telemetry.addData("Output Servo Initialization Complete", "");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Output Motor Initialization Complete", "");
    }

    private void setLiftPower(double liftPower) {
        motorLift.setPower(liftPower);
        telemetry.addData("Lift Power", "" + liftPower);
        telemetry.update();
    }

    public void startMoveLiftUp() {
        setLiftPower(LIFT_POWER_UP);
    }

    public void startMoveLiftDown() {
        setLiftPower(LIFT_POWER_DOWN);
    }

    public void stopLift() {
        setLiftPower(0);
    }

//
//
//    public void moveToPosition(int newPos)
//    {
//        if (newPos == 1)
//        {
//            // inside
//
//            telemetry.addData("" + position, "");
//
//            if (position == 3)
//            {
//
//                threeToTwo();
//            }
//
//            twoToOne();
//
//            position = 1;
//
//            telemetry.addData("" + position, "");
//            telemetry.update();
//        }
//
//        else if (newPos == 2)
//        {
//            // outside high
//
//            telemetry.addData("" + position, "");
//
//            // go outward from position 1
//            if (position == 1)
//                oneToTwo();
//
//            // raise from position 3
//            if (position == 3)
//                threeToTwo();
//
//            position = 2;
//
//            telemetry.addData("" + position, "");
//            telemetry.update();
//
//        }
//
//        else if (newPos == 3)
//        {
//            // outside low
//            // lower from position 2;
//
//            telemetry.addData("" + position, "");
//
//            if (position == 1) {
//                oneToTwo();
//
//                setLiftPower(-0.5);
//                sleep(1000);
//                setLiftPower(0);
//
//                twoToThree();
//            }
//
//            if (position == 2)
//                twoToThree();
//
//            position = 3;
//
//            telemetry.addData("" + position, "");
//            telemetry.update();
//        }
//    }
//
//    // since intake not working right now, this will only be used in auto to go out
//    public void oneToTwo() {
//        closeClamp();
//        sleep(1000);
//
//        setLiftPower(0.5);
//        sleep(1000);
//        setLiftPower(0);
//
//        setElbowPositions(0.3);
//        wrist.setPosition(0.5); // 90 degrees
//        setElbowPositions(0.7);
//        wrist.setPosition(1);
//    }
//
//    // won't use in AML2
//    public void twoToOne() {
//        setLiftPower(0.5);
//        sleep(1000);
//        setLiftPower(0);
//
//        wrist.setPosition(0.5);
//        setElbowPositions(0.3);
//        wrist.setPosition(0);
//
//        setLiftPower(-0.5);
//        sleep(1000);
//        setLiftPower(0);
//
//        setElbowPositions(0);
//
//        openClamp();
//        sleep(500);
//    }
//
//    public void threeToTwo() {
//        closeClamp();
//        sleep(1000);
//
//        setElbowPositions(0.7);
//    }
//
//    public void twoToThree()
//    {
//        setElbowPositions(1);
//
//        openClamp();
//        sleep(1000);
//    }

    public void openClamp()
    {
        clamp.setPower(1);
    }

    public void closeClamp()
    {
        clamp.setPower(-1);
    }

    public void stopClamp()
    {
        clamp.setPower(0);
    }

    // This needs to be faster than 5 seconds.
    public void moveClampOutOfRobot()
    {
        if (! isClampInRobot())
        {
            telemetry.addData("WARNING. Clamp is not in robot!","");
            telemetry.update();
            return;
        }

        // Raise the lift some.
        startMoveLiftUp();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        moveWristToPosition(WRIST_POSITION_SIDEWAYS);

        moveElbowToPosition(ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);

        moveWristToPosition(WRIST_POSITION_OUTSIDE_ROBOT);

        // Lower the lift back down.
        startMoveLiftDown();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();
    }

    public void moveClampIntoRobot()
    {
        if (! isClampOutOfRobot())
        {
            telemetry.addData("WARNING. Clamp is not out of robot!","");
            telemetry.update();
            return;
        }

        // Raise the lift some.
        startMoveLiftUp();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        moveWristToPosition(WRIST_POSITION_SIDEWAYS);

        moveElbowToPosition(ELBOW_POSITION_INSIDE_ROBOT);

        moveWristToPosition(WRIST_POSITION_INSIDE_ROBOT);

        // Lower the lift back down.
        startMoveLiftDown();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();
    }

    private boolean isClampInRobot()
    {
        return elbowL.getPosition() < 0.1;
    }

    private boolean isClampOutOfRobot()
    {
        return elbowL.getPosition() > 0.7;
    }

    private void moveWristToPosition(double pos)
    {
        wrist.setPosition(pos);

        sleep(1000);
    }

    private void moveElbowToPosition(double pos)
    {
        setElbowPositions(pos);

        sleep(1000);
    }


    public void setElbowPositions(double pos)
    {
        telemetry.addData("Move to", pos + "");
        telemetry.addData("At", elbowL.getPosition() + "");
        telemetry.update();

        double posL = pos;
        double posR = 1 - pos;

        elbowL.setPosition(posL);
        elbowR.setPosition(posR);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}