package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    private static final int MOVE_AWAY_FROM_WALL_INCHES = 3;
    private static final int STRAFE_ALONG_WALL_INCHES = 24;
    private static final int MOVE_FORWARD_TO_PLATFORM_INCHES = 29;
    private static final int MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES = 5;
    private static final int STRAFE_PLATFORM_AWAY_FROM_WALL_INCHES = 14;
    private static final int STRAFE_PLATFORM_TO_CORNER_INCHES = 8;
    private static final int MOVE_BACKWARD_FROM_PLATFORM_INCHES = 3;

    private static final int STRAFE_TO_SIDE_TO_PARK_INCHES = 21;

    // new - was 36 before
    private static final int MOVE_BACKWARD_TO_PARK_INCHES = 34;
    private static final int STRAFE_TO_CENTER_TO_PARK_INCHES = 15;


    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    public void initialize()
    {
        super.initialize();
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
        moveForwardByInches(0.25, MOVE_AWAY_FROM_WALL_INCHES);
        strafeRightByInches(0.5, STRAFE_ALONG_WALL_INCHES);
        moveForwardByInches(0.5, MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.5, MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.hooks.lowerHooks();

        moveBackwardByInches(0.5, 32);

        // new
        strafeLeftByInches(0.5, 12);

        rotateToAbsoluteYaw(90);

        robot.hooks.raiseHooks();

        moveBackwardByInches(0.5, MOVE_BACKWARD_FROM_PLATFORM_INCHES);
    }



    private void moveBlueFoundation()
    {
        moveForwardByInches(0.25, MOVE_AWAY_FROM_WALL_INCHES);
        strafeLeftByInches(0.5, STRAFE_ALONG_WALL_INCHES);
        moveForwardByInches(0.5, MOVE_FORWARD_TO_PLATFORM_INCHES);

        // Make sure we are hitting the platform.
        moveForwardByInches(0.5, MOVE_FORWARD_TO_TOUCH_PLATFORM_INCHES);

        robot.hooks.lowerHooks();

        moveBackwardByInches(0.5, 32);

        strafeRightByInches(0.5, 12);

        rotateToAbsoluteYaw(-90);

        robot.hooks.raiseHooks();

        moveBackwardByInches(0.5, MOVE_BACKWARD_FROM_PLATFORM_INCHES);
    }



     public void runSideRed()
    {
        log("runSideRed");
        sleep(1000);

        moveRedFoundation();

        strafeRightByInches(0.5, 12);

        moveBackwardByInches(0.5, MOVE_BACKWARD_TO_PARK_INCHES);
    }





    public void runSideBlue() {
        log("runSideBlue");
        sleep(1000);

        moveBlueFoundation();

        strafeLeftByInches(0.5, 12);

        moveBackwardByInches(0.5, MOVE_BACKWARD_TO_PARK_INCHES);
    }



    public void runCenterRed() {
        log("runCenterRed");
        sleep(1000);

        moveRedFoundation();

        strafeLeftByInches(0.5, STRAFE_TO_CENTER_TO_PARK_INCHES);
        moveBackwardByInches(0.5, MOVE_BACKWARD_TO_PARK_INCHES);
    }



    public void runCenterBlue() {

        log("runCenterBlue");
        sleep(1000);

        moveBlueFoundation();

        strafeRightByInches(0.5, STRAFE_TO_CENTER_TO_PARK_INCHES);
        moveBackwardByInches(0.5, MOVE_BACKWARD_TO_PARK_INCHES);
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