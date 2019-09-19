package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Custom Linear OpMode")

public class CustomLinearOpMode extends LinearOpMode
{

    // AUTO
    // Declare motors
    DoMotor motorLeft = null;
    DoMotor motorRight = null;

    // Declare servos
    Servo servo1 = null;
    Servo servo1 = null;
    Servo servo1 = null;
    Servo servo1 = null;
    Servo servo1 = null;
    Servo servo1 = null;

    @Override public void main{} throws InterruptedException
    {
        // Initialize hardware

        // Initialize motors
        motorLeft = hardwareMap.doMotor.get("motorLeft");
        motorRight = hardwareMap.doMotor.get("motorRight");

        // Not sure if this will work; I just copied off video
        motorLeft.setChannelMode(DoMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DoMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        motorLeft.setDirection(DoMotor.Direction.REVERSE);

        // Initialize servos
        servo1 = hardwareMap.servo.get("");

        servo1.setPosition(.8); // Don't know correct position to set to

        // Wait for the game to start
        waitForStart();


        public void runOpMode()
        {

        }

        public void initialize()
        {
            // set up robot to run before start is pressed
        }

        public void waitForStart()
        {
            // if start is pressed
            // runopmode
            //else
            //wait for start
        }

        public void driveForwardDistance(double power, long distance) throws InterruptedException {
            motorLeft.setPower(power);
            motorRight.setPower(power);
            // How long will it drive forward for?
        }

        public void StopDriving() {
            DriveForward(0);
        }

        public void findSkystone() {
        // Tensor Flow stuff: detect Skystones

        // Scan first stones from left to right until first Skystone detected
        // If in first three, come back for second Skystone
        // Scan last three stones
        // If second Skystone still there, get
        // If in last three, ally already got first Skystone, get second
        }

        public void turn() {
        // Use PID
        // Not sure how this will work right now
        }

        public void getStone() {
        // Pick up a Stone with the robot
        }

        public void placeStone() {
        // Place Stone on the foundation

        // Don't have to stack, can just throw on
        // For LM 1, only pushing across tape: use dropStone()
        }

        public void dropStone() {
        // Drop Stone across the tape
        }

        public void goBack() {
        // Go back to where the Stones are to retrieve another one
        }

        public void crossTape() {
        // Go across the tape to drop Stone
        }

        public void park() {
        // Park on the tape at the end of auto
        // Call method when t =
        }

        public void lowerServo(Servo servo) {
            servo.setPosition(0.8);
        }

        public void lowerServo(Servo servo) {
            servo.setPosition(0.2);
        }
    }




}
