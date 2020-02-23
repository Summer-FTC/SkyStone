package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{
    // Positions for the wrist servo.
    public static final double WRIST_POSITION_SIDEWAYS = 0.66;
    public static final double WRIST_POSITION_INSIDE_ROBOT = 0;
    public static final double WRIST_POSITION_OUTSIDE_ROBOT= 0.97;
    public static final long WRIST_POSITION_DURATION = 200;
    public static final long ELBOW_POSITION_DURATION = 2500;

    // Positions for the elbow servo.
    public static final double ELBOW_POSITION_INSIDE_ROBOT = 0;
    public static final double ELBOW_POSITION_OUTSIDE_ROBOT_PARALLEL = 0.9;
    public static final double ELBOW_POSITION_OUTSIDE_ROBOT_DOWN_A_LITTLE = 0.95;
    public static final double ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN = 1;
    public static final double ELBOW_POSITION_OUTSIDE_ROBOT_AND_PARTIALLY_UP = 0.75;

    public static final double LIFT_POWER_UP = -1;
    public static final double LIFT_TENSION_POWER = -0.15;
    public static final long LIFT_TENSION_DURATION = 1000;
    // This is negative to move down and has a much smaller absolute value since gravity
    // helps us down.
    // When using DcMotor.ZeroPowerBehavior.BRAKE, this should be larger like 0.8.
    private static final double LIFT_POWER_DOWN = 0.5;

    // How long to move the lift up and then down when moving the clamp in or out of the robot.
    private static final long MOVE_CLAMP_LIFT_DURATION = 150;

    private static final long OPEN_CLAMP_FULLY_DURATION = 1500; // This isn't fully.
    private static final long CLOSE_CLAMP_FULLY_DURATION = 3200;

    public static final double RIGHT_DOOR_FULLY_OPEN = 0.9;
    public static final double LEFT_DOOR_FULLY_OPEN = 0.1;

    public static final double RIGHT_DOOR_PARTWAY_OPEN = 0.37;
    public static final double LEFT_DOOR_PARTWAY_OPEN = 0.4;

    public static final double RIGHT_DOOR_CLAMPING_BLOCK = 0.25;
    public static final double LEFT_DOOR_CLAMPING_BLOCK = 0.59;

    public DcMotor motorLift;
    public Servo elbowR;
    public Servo elbowL;
    public Servo wrist;
    public CRServo clamp;

    public Servo rightDoor;
    public Servo leftDoor;
    public Servo cap;


    HardwareMap hwMap;
    Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry, boolean isAuto)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");

        // elbows flip
        // 1 and 2 should work simultaneously
     //   elbowR = hwMap.servo.get("elbow1");
     //   elbowL = hwMap.servo.get("elbow2");

        // wrist rotates block
     //   wrist = hwMap.servo.get("wrist");
        // clamp opens and closes on block
     //   clamp = hwMap.crservo.get("clamp");

        rightDoor = hwMap.servo.get("rightDoor");
        leftDoor = hwMap.servo.get("leftDoor");
        cap = hwMap.servo.get("cap");


  //      telemetry.addData("Output Servo Initialization Complete", "");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (isAuto) {
            motorLift.setPower(LIFT_TENSION_POWER);
            sleep(LIFT_TENSION_DURATION);
            motorLift.setPower(0);
        }

        // This MIGHT have caused the chain to break. Disable it just in case for now.
//        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // stops lift from drifting down

//        telemetry.addData("Output Motor Initialization Complete", "");
//        telemetry.update();
    }

    private void setLiftPower(double liftPower) {
        motorLift.setPower(liftPower);
//        if (liftPower != 0) {
//            telemetry.addData("Lift Power", liftPower);
//            telemetry.addData("Lift encoder", motorLift.getCurrentPosition());
//            telemetry.update();
//        }
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

    public void shutDoors(){
        leftDoor.setPosition(LEFT_DOOR_CLAMPING_BLOCK);
        rightDoor.setPosition(RIGHT_DOOR_CLAMPING_BLOCK);
    }

    public void openDoorsPartyway(){
        leftDoor.setPosition(LEFT_DOOR_PARTWAY_OPEN);
        rightDoor.setPosition(RIGHT_DOOR_PARTWAY_OPEN);
    }

    public void openDoorsFully(){
        leftDoor.setPosition(LEFT_DOOR_FULLY_OPEN);
        rightDoor.setPosition(RIGHT_DOOR_FULLY_OPEN);
    }

    public void releaseCap() {
        cap.setPosition(0);
    }

    public void pullCapBackIn(){
        cap.setPosition(1);
    }


    public void openClampFully() {
        startOpeningClamp();
        sleep(OPEN_CLAMP_FULLY_DURATION);
        stopClamp();
    }

    public void closeClampFully() {
        startClosingClamp();
        sleep(CLOSE_CLAMP_FULLY_DURATION); // This is a little bigger than on open to be sure we close on the stone.
        stopClamp();
    }

    public void closeClampPartway() {
        // Closes clamp enough to grab a stone longways
        startClosingClamp();
        sleep(1000);
        stopClamp();
    }

    public void startOpeningClamp()
    {
        clamp.setPower(1);
    }

    public void startClosingClamp()
    {
        clamp.setPower(-1);
    }

    public void stopClamp()
    {
        clamp.setPower(0);
    }

    private void printElbows(String when, StringBuilder history) {
//        history.append(when + " L elbow=" + getLeftElbowPos() + "  R elbow=" + getRightElbowPos() + "\n");
//        telemetry.addData(history.toString(), "");
//        telemetry.update();
    }

    /**
     * This is just to demo moving out the clamp in the most impressive way possible.
     * With the output controller pull either joystick over and while it's held down, push Y.
     */
    public void moveClampOutOfRobotFromUnderHooks(VenomRobot robot)
    {
        long startMillis = System.currentTimeMillis();


        robot.stoneHooks.leftStoneHook.setPosition(0.5); // 1 is raised and 0 is lowered
        robot.stoneHooks.rightStoneHook.setPosition(0.5); // 0 is raised and 1 is lowered.
        sleep(600);

        // Raise the lift some.
        startMoveLiftUp();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        moveWristToPosition(WRIST_POSITION_SIDEWAYS);
        startOpeningClamp();

        moveElbowToPosition(ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);

        robot.stoneHooks.startRaiseHooks();

        moveWristToPosition(WRIST_POSITION_OUTSIDE_ROBOT);

        // Lower the lift back down.
        startMoveLiftDown();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        long durationMillis = System.currentTimeMillis() - startMillis;

        sleep(OPEN_CLAMP_FULLY_DURATION - durationMillis);
        stopClamp();

        telemetry.addData("Moving clamp out took " + durationMillis + " ms", "");
        telemetry.update();
    }

    public void moveClampOutOfRobot()
    {
        long startMillis = System.currentTimeMillis();


        // Raise the lift some.
        startMoveLiftUp();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();


        moveElbowToPosition(ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);

        startOpeningClamp();
        moveWristToPosition(WRIST_POSITION_OUTSIDE_ROBOT);

        // Lower the lift back down.
        startMoveLiftDown();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        long durationMillis = System.currentTimeMillis() - startMillis;

        sleep(OPEN_CLAMP_FULLY_DURATION - durationMillis);
        stopClamp();

        telemetry.addData("Moving clamp out took " + durationMillis + " ms", "");
        telemetry.update();
    }

    public void moveClampIntoRobot()
    {
        if (! isClampOutOfRobot())
        {
            telemetry.addData("WARNING. Clamp is not in robot!" +
                    " L elbow=" + getLeftElbowPos() + "  R elbow=" + getRightElbowPos(), "");
            telemetry.update();
            return;
        }

        // Raise the lift some.
        startMoveLiftUp();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();

        moveWristToPosition(WRIST_POSITION_SIDEWAYS);

        moveElbowToPosition(ELBOW_POSITION_INSIDE_ROBOT);

        // Lower the lift back down.
        startMoveLiftDown();
        sleep(MOVE_CLAMP_LIFT_DURATION);
        stopLift();
    }

    private boolean isClampInRobot()
    {
        // elbowL is not connected.
//        return getLeftElbowPos() < 0.4;
        // Left starts at 0 and right starts at 1

        return (getRightElbowPos() < 0.4 || getLeftElbowPos() < 0.4);
    }

    private boolean isClampOutOfRobot()
    {
        // elbowL is not connected.
//        return getLeftElbowPos() > 0.6;

        return (getRightElbowPos() > 0.6 || getLeftElbowPos() > 0.6);
    }

    public void moveWristToPosition(double pos)
    {
        wrist.setPosition(pos);

        sleep(WRIST_POSITION_DURATION);
    }

    public void moveElbowToPosition(double pos)
    {
        setElbowPositions(pos);

        sleep(ELBOW_POSITION_DURATION);
    }


    private double getLeftElbowPos()
    {
        return elbowL.getPosition();
    }

    private double getRightElbowPos()
    {
        // The right elbow positions are flipped, so we have to convert them.
        return convertRightPosition(elbowR.getPosition());
    }

    private double convertRightPosition(double pos)
    {
        return 1- pos;
    }

    public void setElbowPositions(double pos)
    {
        telemetry.addData("Move to", pos + "");
        telemetry.addData("At", elbowL.getPosition() + "");
        telemetry.update();

        double posL = pos;
        double posR = convertRightPosition(pos);

        elbowL.setPosition(posL);
        elbowR.setPosition(posR);
    }

    public final void sleep(long milliseconds) {
        try {
            if (milliseconds > 0)
                Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}