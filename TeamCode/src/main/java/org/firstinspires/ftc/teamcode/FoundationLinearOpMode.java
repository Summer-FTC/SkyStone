package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    private static final int MOVE_FOUNDATION_STEP_1_MOVE_AWAY_FROM_WALL_INCHES = 3;
    private static final int MOVE_FOUNDATION_STEP_2_STRAFE_ALONG_WALL_INCHES = 20;
    private static final int MOVE_FOUNDATION_STEP_3_MOVE_FORWARD_TO_PLATFORM_INCHES = 29;
    private static final int MOVE_FOUNDATION_STEP_3B_MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES = 2;
    private static final int MOVE_FOUNDATION_STEP_4_MOVE_BACKWARD_FROM_PLATFORM_INCHES = 3;
    private static final int STEP_5_STRAFE_TO_SIDE_PARK_INCHES = 33;
    private static final int STEP_5_STRAFE_TO_CENTER_PARK_INCHES = 26;
    private static final int STEP_6_MOVE_FORWARD_TO_CENTER_PARK_INCHES = 16;
    private static final int STEP_7_STRAFE_LEFT_TO_CENTER_PARK_INCHES = 18;

    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;


    public void initialize()
    {
        super.initialize();
        robot.hooks.raiseHooks();
    }



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

        moveForwardByInches(0.25, MOVE_FOUNDATION_STEP_1_MOVE_AWAY_FROM_WALL_INCHES);
        strafeRightByInches(0.5, MOVE_FOUNDATION_STEP_2_STRAFE_ALONG_WALL_INCHES);
        moveForwardByInches(0.5, MOVE_FOUNDATION_STEP_3_MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.5, MOVE_FOUNDATION_STEP_3B_MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.hooks.lowerHooks();
        strafeLeftByInches(0.5,  14); // TODO: Make a constant
        rotateToAbsoluteYaw(-90);

        strafeRightByInches(0.5, 8); // TODO: Make a constant
        rotateToAbsoluteYaw(-90);

        robot.hooks.raiseHooks();
        moveBackwardByInches(0.5, MOVE_FOUNDATION_STEP_4_MOVE_BACKWARD_FROM_PLATFORM_INCHES);
    }



    private void moveBlueFoundation()
    {
        moveForwardByInches(0.25, MOVE_FOUNDATION_STEP_1_MOVE_AWAY_FROM_WALL_INCHES);
        strafeLeftByInches(0.5, MOVE_FOUNDATION_STEP_2_STRAFE_ALONG_WALL_INCHES);
        moveForwardByInches(0.5, MOVE_FOUNDATION_STEP_3_MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.5, MOVE_FOUNDATION_STEP_3B_MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.hooks.lowerHooks();
        strafeRightByInches(0.5,  14); // TODO: Make a constant
        rotateToAbsoluteYaw(-90);

        strafeLeftByInches(0.5, 8); // TODO: Make a constant
        rotateToAbsoluteYaw(-90);

        robot.hooks.raiseHooks();
        moveBackwardByInches(0.5, MOVE_FOUNDATION_STEP_4_MOVE_BACKWARD_FROM_PLATFORM_INCHES);
    }

     public void runSideRed()
    {
        log("runSideRed");
        sleep(1000);

        moveRedFoundation();

        strafeRightByInches(0.5, 21);    // TODO: Constants
        moveBackwardByInches(0.5, 37);  // TODO: Constants
    }



    public void runCenterRed() {
        log("runCenterRed");
        sleep(1000);

        moveRedFoundation();

        moveBackwardByInches(0.5, 37);  // TODO: Constants
        strafeRightByInches(0.5, 5);    // TODO: Constants
    }



    public void runSideBlue() {
        log("runSideBlue");
        sleep(1000);

        moveBlueFoundation();

        strafeLeftByInches(0.5, 21);    // TODO: Constants
        moveBackwardByInches(0.5, 37);  // TODO: Constants
    }



    public void runCenterBlue() {

        log("runCenterBlue");
        sleep(1000);

        moveBlueFoundation();

        moveBackwardByInches(0.5, 37);  // TODO: Constants
        strafeLeftByInches(0.5, 5);    // TODO: Constants
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