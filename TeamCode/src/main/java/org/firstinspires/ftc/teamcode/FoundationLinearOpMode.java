package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    @Override
    public void initialize() {
        super.initialize();
        robot.hooks.lowerHooks(false);
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

//    public void runSideRed() {
//        moveForwardByInches(0.5, 3000, 2);
//        strafeRightByInches(0.5, 3000, 30);
//        moveForwardByInches(0.5, 3000, 29);
//
//        robot.hooks.lowerHooks(true);
//        sleep(2000);
//        moveBackwardByInches(0.5, 3000, 33);
//        // maybe turn sideways
//        robot.hooks.lowerHooks(false);
//        sleep(2000);
//
//        // wait before parking
//
//        strafeLeftByInches(0.5, 10000, 70);
//    }

    public void runSideRed() {
        moveForwardByInches(0.5, 3000, 2);
        strafeRightByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 29);

        robot.hooks.lowerHooks(true);
        sleep(2000);
        strafeLeftByInches(0.5, 3000, 20);
        rotate(180);

        moveForwardByInches(0.5, 3000, 6);

        // maybe turn sideways
        robot.hooks.lowerHooks(false);
        sleep(2000);

        // wait before parking
        moveBackwardByInches(0.25, 3000, 3);
        strafeRightByInches(0.5, 10000, 70);
    }

    public void runCenterRed() {
        moveForwardByInches(0.5, 3000, 1);
        strafeRightByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        robot.hooks.lowerHooks(true);
        sleep(2000);
        moveBackwardByInches(0.5, 3000, 30);
        // maybe turn horizontal
        robot.hooks.lowerHooks(false);
        sleep(2000);

        strafeLeftByInches(0.5, 8000, 45);
        moveForwardByInches(0.5, 5000, 22);
        strafeLeftByInches(0.5, 5000, 25);
    }

    public void runSideBlue() {
        moveForwardByInches(0.5, 3000, 1);
        strafeLeftByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        robot.hooks.lowerHooks(true);
        sleep(2000);
        moveBackwardByInches(0.5, 3000, 30);
        // maybe turn sideways
        robot.hooks.lowerHooks(false);
        sleep(2000);

        strafeRightByInches(0.5, 10000, 70);
    }

    public void runCenterBlue() {
        moveForwardByInches(0.5, 3000, 1);
        strafeLeftByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        robot.hooks.lowerHooks(true);
        sleep(2000);
        moveBackwardByInches(0.5, 3000, 30);
        // maybe turn sideways
        robot.hooks.lowerHooks(false);
        sleep(2000);

        strafeRightByInches(0.5, 8000, 45);
        moveForwardByInches(0.5, 5000, 22);
        strafeRightByInches(0.5, 5000, 25);
    }

    public void pullFoundation()
    {
        robot.hooks.lowerHooks(true);
        moveBackwardByInches(0.5, 3000, 28);
        // maybe turn sideways
        robot.hooks.lowerHooks(false);
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