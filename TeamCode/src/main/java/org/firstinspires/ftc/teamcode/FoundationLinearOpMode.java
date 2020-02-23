package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    private static final int MOVE_AWAY_FROM_WALL_INCHES = 3;
    private static final int MOVE_FORWARD_TO_PLATFORM_INCHES = 25;
    private static final int MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES = 3;
    private static final int MOVE_BACKWARD_TO_PARK_INCHES1 = 10;
    private static final int MOVE_BACKWARD_TO_PARK_INCHES2 = 30;
    private static final int STRAFE_TO_CENTER_TO_PARK_INCHES = 26;


    private static final int MAX_PARKING_DURATION_MILLIS = 5_000;

    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    public void initialize()
    {
        super.initialize(true);
    }


    // TODO: do we need to move forward more when hitting the foundation? it seemed like we cut it kinda close last time

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();
        initialize();
        configMode();

        waitForStart();

        // Autos are 30 seconds, so we work backwards from that to decide when to park.
        // We wait as long as possible to stay out of the way of a robot that's bringing over
        // stones.
        long parkTimeMillis = System.currentTimeMillis() + (30_000 - MAX_PARKING_DURATION_MILLIS);

        if (!isStartingBlue && parkOnSide) {
            runSideRed();
        } else if (!isStartingBlue && !parkOnSide) {
            runCenterRed();
        } else if (isStartingBlue && parkOnSide) {
            runSideBlue();
        } else if (isStartingBlue && !parkOnSide) {
            runCenterBlue();
        }

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES1);
        //moveClampOutInAuto();

        long millisUntilPark = parkTimeMillis - System.currentTimeMillis();
        if (millisUntilPark > 0) {
            log("Sleeping " + millisUntilPark + " before parking to give other robot room.");
            sleep(millisUntilPark);
        }

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES2);

        telemetry.addData("After running", "");
        telemetry.update();
    }


    private void moveRedFoundation()
    {
        moveForwardByInches(0.8, MOVE_AWAY_FROM_WALL_INCHES);
        strafeRightByInches(0.9, 12);
        moveForwardByInches(1, MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.9, MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.foundationHooks.lowerHooks();

        strafeLeftByInches(0.75, 8);

        arcBackwardsToAbsoluteYaw(0.8, 15, 30, false);
        rotateToAbsoluteYaw(90);
        moveForwardByInches(0.8, 10);

        robot.foundationHooks.raiseHooks();
    }

    private void moveBlueFoundation() {
        moveForwardByInches(0.8, MOVE_AWAY_FROM_WALL_INCHES);
        strafeLeftByInches(0.9, 12);
        moveForwardByInches(1, MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.9, MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.foundationHooks.lowerHooks();

        strafeRightByInches(0.75, 8);

        arcBackwardsToAbsoluteYaw(0.8, 15, -30, true);
        rotateToAbsoluteYaw(-90);
        moveForwardByInches(0.8, 10);

        robot.foundationHooks.raiseHooks();
    }


    public void runSideRed()
    {
        log("runSideRed");

        moveRedFoundation();

        strafeRightByInches(0.7, 20);
    }


    public void runSideBlue() {
        log("runSideBlue");

        moveBlueFoundation();

        strafeLeftByInches(0.7, 20);
    }


    public void runCenterRed() {
        log("runCenterRed");

        moveRedFoundation();

        // Strafe into the wall to reset the distances
        strafeRightByInches(0.7, 18);

        strafeLeftByInches(0.7, STRAFE_TO_CENTER_TO_PARK_INCHES);
    }


    public void runCenterBlue() {

        log("runCenterBlue");

        moveBlueFoundation();

        // Strafe into the wall to reset the distances
        strafeLeftByInches(0.7, 18);

        strafeRightByInches(0.7, STRAFE_TO_CENTER_TO_PARK_INCHES);
    }


    public void configMode() {
        telemetry.addData("Entering " , "ConfigMode");
        telemetry.update();
        do {
            if (VenomUtilities.isValueChangedAndEqualTo("1.y", gamepad1.y, true))
                isStartingBlue = !isStartingBlue;

            if (VenomUtilities.isValueChangedAndEqualTo("1.a", gamepad1.a, true))
                parkOnSide = !parkOnSide;

            logConfigModes(true);
        }

        while (!gamepad1.right_bumper && !isStarted() &&  !isStopRequested());

        String lastModes = "";
        lastModes+=", Alliance="+(isStartingBlue?"Blue":"Red");
        lastModes+=", Park="+(parkOnSide?"Side":"Center");

        telemetry.addData("Final ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
    }


    private String lastModes="";
    void logConfigModes(boolean update) {
        String modes="";
        //  modes+="Alliance="+(isRedAlliance?"Red":"Blue");
        modes+=", Alliance="+(isStartingBlue?"Blue":"Red");
        modes+=", Park="+(parkOnSide?"Side":"Center");


        telemetry.addData("Alliance (Y)", isStartingBlue?"Blue":"Red");
        telemetry.addData("Park Location (A)", parkOnSide?"Side":"Center");

        if (configOnly) telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        if (update) telemetry.update();

        if (!modes.equals(lastModes)) {
            RobotLog.i(modes);
            lastModes=modes;
        }
    }
}