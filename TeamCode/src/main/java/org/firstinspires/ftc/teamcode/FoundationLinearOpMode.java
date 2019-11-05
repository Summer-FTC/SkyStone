package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{

    private static final int STEP_1_MOVE_FORWARD_INCHES = 2;
    private static final int STEP_2_STRAFE_INCHES = 30;
    private static final int STEP_3_MOVE_FORWARD_INCHES = 29;
    private static final int STEP_4_MOVE_BACKWARD_INCHES = 34;
    private static final int STEP_5_STRAFE_TO_SIDE_PARK_INCHES = 70;
    private static final int STEP_5_STRAFE_TO_CENTER_PARK_INCHES = 45;
    private static final int STEP_6_MOVE_FORWARD_TO_CENTER_PARK_INCHES = 26;
    private static final int STEP_7_STRAFE_LEFT_TO_CENTER_PARK_INCHES = 25;

    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    @Override
    public void initialize()
    {
        super.initialize();
        robot.hooks.raiseHooks();
    }

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
//        if (configOnly) {
//            readConfigValues();
//            configMode();
//            writeConfigValues();
//            logConfigModes(false);
//        } else {
//            telemetry.addData("Status", "Initializing...");
//            telemetry.update();
//
//
//
//            telemetry.addData("Status", "Initialized.");
//            telemetry.update();
//
////            readConfigValues();
//            configMode();
//            writeConfigValues();
//            logConfigModes(false);
//
//            telemetry.addData("Status", "Configuration loaded.");
//            telemetry.update();
//
//
//            telemetry.addData("Status", "Everything Ready. Waiting for Start.");
//            telemetry.update();
//
//
//            while (!(isStarted() || isStopRequested())) { // equivalent to waitForStart();
//                telemetry.addData("Status", "waiting for start");
//                telemetry.update();
//                idle();
//            }
//
            telemetry.addData("Status", "Running Autonomous!");
            telemetry.update();
            initialize();
            configMode();
            waitForStart();

            if (!isStartingBlue && parkOnSide)
                runSideRed();
            else if (!isStartingBlue && !parkOnSide)
                runCenterRed();
            else if (isStartingBlue && parkOnSide)
                runSideBlue();
            else if (isStartingBlue && !parkOnSide)
                runCenterBlue();

            telemetry.addData("After running", "");
            telemetry.update();
    }

    private void moveRedFoundation()
    {
        moveForwardByInches(0.5, 3000, STEP_1_MOVE_FORWARD_INCHES);
        strafeRightByInches(0.5, 3000, STEP_2_STRAFE_INCHES);
        moveForwardByInches(0.5, 3000, STEP_3_MOVE_FORWARD_INCHES);

        robot.hooks.lowerHooks();
        sleep(1000);
        moveBackwardByInches(0.5, 3000, STEP_4_MOVE_BACKWARD_INCHES);
        // maybe turn sideways
        robot.hooks.raiseHooks();
        sleep(1000);
    }

    public void runSideRed()
    {
        log("runSideRed");
        sleep(1000);

        moveRedFoundation();

        // wait before parking?

        strafeLeftByInches(0.5, 10000, STEP_5_STRAFE_TO_SIDE_PARK_INCHES);
    }

    // Attempt rotate.
//    public void runSideRed() {
//        moveForwardByInches(0.5, 3000, 2);
//        strafeRightByInches(0.5, 3000, 30);
//        moveForwardByInches(0.5, 3000, 29);
//
//        robot.hooks.lowerHooks(true);
//        sleep(2000);
//        strafeLeftByInches(0.5, 3000, 20);
//        rotate(180);
//
//        moveForwardByInches(0.5, 3000, 6);
//
//        // maybe turn sideways
//        robot.hooks.lowerHooks(false);
//        sleep(2000);
//
//        // wait before parking
//        moveBackwardByInches(0.25, 3000, 3);
//        strafeRightByInches(0.5, 10000, 70);
//    }



    public void runCenterRed() {
        log("runCenterRed");
        sleep(1000);

        moveRedFoundation();

        strafeLeftByInches(0.5, 8000, STEP_5_STRAFE_TO_CENTER_PARK_INCHES);
        moveForwardByInches(0.5, 5000, STEP_6_MOVE_FORWARD_TO_CENTER_PARK_INCHES);
        strafeLeftByInches(0.5, 5000, STEP_7_STRAFE_LEFT_TO_CENTER_PARK_INCHES);
    }

    private void moveBlueFoundation()
    {
        moveForwardByInches(0.5, 3000, STEP_1_MOVE_FORWARD_INCHES);
        strafeLeftByInches(0.5, 3000, STEP_2_STRAFE_INCHES);
        moveForwardByInches(0.5, 3000, STEP_3_MOVE_FORWARD_INCHES);

        robot.hooks.lowerHooks();
        sleep(1000);
        moveBackwardByInches(0.5, 3000, STEP_4_MOVE_BACKWARD_INCHES);
        // maybe turn sideways
        robot.hooks.raiseHooks();
        sleep(1000);
    }

    public void runSideBlue() {
        log("runSideBlue");
        sleep(1000);

        moveBlueFoundation();

        strafeRightByInches(0.5, 10000, STEP_5_STRAFE_TO_SIDE_PARK_INCHES);
    }

    public void runCenterBlue() {

        log("runCenterBlue");
        sleep(1000);

        moveBlueFoundation();

        strafeRightByInches(0.5, 8000, STEP_5_STRAFE_TO_CENTER_PARK_INCHES);
        moveForwardByInches(0.5, 5000, STEP_6_MOVE_FORWARD_TO_CENTER_PARK_INCHES);
        strafeRightByInches(0.5, 5000, STEP_7_STRAFE_LEFT_TO_CENTER_PARK_INCHES);
    }

    public void pullFoundation()
    {
        robot.hooks.lowerHooks();
        moveBackwardByInches(0.5, 3000, 28);
        // maybe turn sideways
        robot.hooks.raiseHooks();
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