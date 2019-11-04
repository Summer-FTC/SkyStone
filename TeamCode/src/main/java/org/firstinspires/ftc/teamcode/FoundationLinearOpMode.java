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

    public void runSideRed() {
        moveForwardByInches(0.5, 3000, 1);
        strafeRightByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        // deploy hooks

        moveBackwardByInches(0.5, 3500, 28);

        // maybe turn to horizontal

        // release hooks

        strafeLeftByInches(0.5, 10000, 70);
    }

    public void runCenterRed() {
        moveForwardByInches(0.5, 3000, 1);
        strafeRightByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        // deploy hooks

        moveBackwardByInches(0.5, 3000, 28);

        // release hooks

        strafeLeftByInches(0.5, 8000, 45);
        moveForwardByInches(0.5, 5000, 22);
        strafeLeftByInches(0.5, 5000, 25);
    }

    public void runSideBlue() {
        moveForwardByInches(0.5, 3000, 1);
        strafeLeftByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        // deploy hooks

        moveBackwardByInches(0.5, 3500, 28);

        // maybe turn to horizontal

        // release hooks

        strafeRightByInches(0.5, 10000, 70);
    }

    public void runCenterBlue() {
        moveForwardByInches(0.5, 3000, 1);
        strafeLeftByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        // deploy hooks

        moveBackwardByInches(0.5, 3000, 28);

        // release hooks

        strafeRightByInches(0.5, 8000, 45);
        moveForwardByInches(0.5, 5000, 22);
        strafeRightByInches(0.5, 5000, 25);
    }

    public void pullFoundation()
    {
        robot.hooks.lowerHooks(true);
        // pull back
        sleep(2500);
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