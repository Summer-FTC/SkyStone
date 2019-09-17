package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class CustomLinearOpMode extends LinearOpMode
{

    // AUTO

    // set motor names

    public void runOpMode()
    {
    //
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

    public void goStraight(int distance) {
        // To move straight a certain distance
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


}
