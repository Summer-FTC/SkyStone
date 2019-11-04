package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    boolean configOnly = false;
//
//
//    // Boolean Values for input
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

//            telemetry.addData("Before moving", "");

//        displayIMU(30000);

            //doTestRunSide();
            doTestRunCenter();
      //      pullFoundation();

            // 1440 ticks for 100 mm/3.937 inches
            telemetry.addData("After moving", "");
            telemetry.update();


       // }




    }

    public void doTestRunSide() {
        moveForwardByInches(0.5, 3000, 1);
        strafeRightByInches(0.5, 3000, 30);
        moveForwardByInches(0.5, 3000, 27);

        // deploy hooks

        moveBackwardByInches(0.5, 3500, 28);

        // maybe turn to horizontal

        // release hooks

        strafeLeftByInches(0.5, 10000, 70);

    }

    public void doTestRunCenter() {
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


    public void moveToFoundation()
    {
        // if red
        // test # of encoder ticks
            // use tele op to figure out
        // We can also use the distance in inches to find out how far we need to go.
        // I think encoders would be a little inaccurate.
    }

    public void pullFoundation()
    {
        robot.hooks.lowerHooks(true);
        // pull back
        sleep(2500);
        robot.hooks.lowerHooks(false);
    }

    public void park() {
        // park robot on line: center or side?
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