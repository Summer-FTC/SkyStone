package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "ParkLinearOpMode" , group = "6209")

public class ParkLinearOpMode extends BaseLinearOpMode
{
    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;

    public void initialize() {
        super.initialize(true);
    }

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
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
        // So we don't get in the other team's way
        sleep(20000);
        moveForwardByInches(0.5, 1);
        strafeRightByInches(0.5, 10);
    }

    // red is right
    public void runCenterRed() {
        sleep(20000);
        moveForwardByInches(0.5,25);
        strafeRightByInches(0.5,10);
    }

    public void runSideBlue() {
        // So we don't get in the other team's way
        sleep(20000);
        moveForwardByInches(0.5, 1);
        strafeLeftByInches(0.5, 10);
    }

    // blue is left
    public void runCenterBlue() {
        sleep(20000);
        moveForwardByInches(0.5, 25);
        strafeLeftByInches(0.5, 10);
    }

    public void configMode()
    {
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