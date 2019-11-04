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
//
//    @Autonomous(name = "FoundationConfig", group = "6209")
//    public static class FoundationConfig extends FoundationLinearOpMode {
//        public FoundationConfig () {configOnly = true;}
//    }
//
//    @Autonomous(name = "Foundation Run", group = "6209")
//    public static class FoundationRun extends FoundationLinearOpMode {
//        public FoundationRun () {configOnly = true;}
//    }

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
            waitForStart();

//            telemetry.addData("Before moving", "");

//        displayIMU(30000);

            doTestRun();
      //      pullFoundation();

            // 1440 ticks for 100 mm/3.937 inches
            telemetry.addData("After moving", "");
            telemetry.update();


       // }
        // What does this initialize do??



    }

    public void doTestRun() {
        moveForwardByInches(0.5, 3000, 1);
        strafeRightByInches(0.5, 3000, 36);
        moveForwardByInches(0.5, 3000, 16);

        // deploy hooks

        moveBackwardByInches(0.5, 3000, 17);

        // release hooks

        strafeLeftByInches(0.5, 3000, 54);

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
        do {
            if (VenomUtilities.isValueChangedAndEqualTo("1.y", gamepad1.y, true))
                isStartingBlue = !isStartingBlue;
        } while (!gamepad1.right_bumper && !isStarted() &&  !isStopRequested());
        telemetry.addData("ConfigMode" , lastModes);
        telemetry.update();

        RobotLog.i("configMode() stop");
        logConfigModes(true);
    }

    private String lastModes="";
    void logConfigModes(boolean update) {
        String modes="";
        //  modes+="Alliance="+(isRedAlliance?"Red":"Blue");
        modes+=", Starting="+(isStartingBlue?"Blue":"Red");


        telemetry.addData("Alliance (X)", isStartingBlue?"Blue":"Red");

        if (configOnly) telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        if (update) telemetry.update();

        if (!modes.equals(lastModes)) {
            RobotLog.i(modes);
            lastModes=modes;
        }


    }

    void readConfigValues(){

        VenomUtilities.readProperties();
        isStartingBlue = VenomUtilities.getPropBoolean("isStartingingBlue", isStartingBlue);

    }

    void writeConfigValues(){

        VenomUtilities.setPropBoolean("isStartingBlue", isStartingBlue);

        VenomUtilities.writeProperties();

    }


}