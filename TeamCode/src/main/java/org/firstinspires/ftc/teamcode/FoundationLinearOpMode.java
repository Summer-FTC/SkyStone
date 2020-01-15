package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    private static final int MOVE_AWAY_FROM_WALL_INCHES = 3;
    private static final int STRAFE_ALONG_WALL_INCHES = 24;
    private static final int MOVE_FORWARD_TO_PLATFORM_INCHES = 25;
    private static final int MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES = 3;
    private static final int STRAFE_PLATFORM_AWAY_FROM_WALL_INCHES = 14;
    private static final int STRAFE_PLATFORM_TO_CORNER_INCHES = 8;
    private static final int MOVE_BACKWARD_FROM_PLATFORM_INCHES = 3;

    private static final int STRAFE_TO_SIDE_TO_PARK_INCHES = 21;


    private static final int MOVE_BACKWARD_TO_PARK_INCHES1 = 20;
    private static final int MOVE_BACKWARD_TO_PARK_INCHES2 = 20;
    private static final int STRAFE_TO_CENTER_TO_PARK_INCHES = 26;


    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    public void initialize()
    {
        super.initialize(true);
    }


    // TODO: do we need to move forward more when hitting the foundation? it seemed like we cut
    // it close a couple of times at AML 3

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();
        initialize();
        configMode();
        waitForStart();

        if (!isStartingBlue && parkOnSide) {
            runSideRed();
        } else if (!isStartingBlue && !parkOnSide) {
            runCenterRed();
        } else if (isStartingBlue && parkOnSide) {
            runSideBlue();
        } else if (isStartingBlue && !parkOnSide) {
            runCenterBlue();
        }

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

        robot.hooks.lowerHooks();

        strafeLeftByInches(0.75, 8);

        arcBackwardsToAbsoluteYaw(0.8, 15, 30, false); // TODO: It seems like this should be true???
        rotateToAbsoluteYaw(90);
        moveForwardByInches(0.8, 10);

        robot.hooks.raiseHooks();
    }

    private void moveBlueFoundation() {
        moveForwardByInches(0.8, MOVE_AWAY_FROM_WALL_INCHES);
        strafeLeftByInches(0.9, 12);
        moveForwardByInches(1, MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.9, MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.hooks.lowerHooks();

        strafeRightByInches(0.75, 8);

        arcBackwardsToAbsoluteYaw(0.8, 15, -30, true);
        rotateToAbsoluteYaw(-90);
        moveForwardByInches(0.8, 10);

        robot.hooks.raiseHooks();
    }


    public void runSideRed()
    {
        log("runSideRed");

        moveRedFoundation();

        strafeRightByInches(0.7, 10);

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES1);
        moveClampOutInAuto();
        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES2);
    }


    public void runSideBlue() {
        log("runSideBlue");

        moveBlueFoundation();

        strafeLeftByInches(0.7, 10);

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES1);
        moveClampOutInAuto();
        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES2);
    }



    public void runCenterRed() {
        log("runCenterRed");

        moveRedFoundation();

        // Strafe into the wall to reset the distances
        strafeRightByInches(0.7, 14);

        strafeLeftByInches(0.7, STRAFE_TO_CENTER_TO_PARK_INCHES);

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES1);
        moveClampOutInAuto();
        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES2);
    }



    public void runCenterBlue() {

        log("runCenterBlue");

        moveBlueFoundation();

        // Strafe into the wall to reset the distances
        strafeLeftByInches(0.7, 14);

        strafeRightByInches(0.7, STRAFE_TO_CENTER_TO_PARK_INCHES);

        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES1);
        moveClampOutInAuto();
        moveBackwardByInches(0.7, MOVE_BACKWARD_TO_PARK_INCHES2);
    }



    public void configMode() {
        String lastModes = "";
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
        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");

    }


    private String lastModes="";
    void logConfigModes(boolean update) {
        String modes="";
        //  modes+="Alliance="+(isRedAlliance?"Red":"Blue");
        modes+=", Starting="+(isStartingBlue?"Blue":"Red");
        modes+=", Starting="+(parkOnSide?"Side":"Center");


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