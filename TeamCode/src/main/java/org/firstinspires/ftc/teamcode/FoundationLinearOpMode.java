package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous(name = "Foundation Run", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{

    LinearOpMode opMode;
    private static final int STEP_1_MOVE_FORWARD_INCHES = 2;
    private static final int STEP_2_STRAFE_INCHES = 32;
    private static final int STEP_3_MOVE_FORWARD_INCHES = 30;
    private static final int STEP_4_MOVE_BACKWARD_INCHES = 34;
    private static final int STEP_5_STRAFE_TO_SIDE_PARK_INCHES = 41;
    private static final int STEP_5_STRAFE_TO_CENTER_PARK_INCHES = 32;
    private static final int STEP_6_MOVE_FORWARD_TO_CENTER_PARK_INCHES = 16;
    private static final int STEP_7_STRAFE_LEFT_TO_CENTER_PARK_INCHES = 22;

    boolean configOnly = false;

    // Boolean Values for input
    boolean isStartingBlue = true;
    boolean parkOnSide = true;


    public void initialize()
    {
        super.initialize(opMode);
        robot.hooks.raiseHooks();
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

    private void moveRedFoundation()
    {
        moveEnc(0.5, 3000, 30);
       // moveForwardByInches(0.5, 3000, STEP_1_MOVE_FORWARD_INCHES);
      //  strafeRightByInches(0.5, 3000, STEP_2_STRAFE_INCHES);
     //   moveForwardByInches(0.5, 3000, STEP_3_MOVE_FORWARD_INCHES);

        robot.hooks.lowerHooks();
        sleep(1000);
        moveBackwardByInches(0.5, 3000, STEP_4_MOVE_BACKWARD_INCHES);

        // fix this
        rotate(-88);

        // maybe turn sideways
        robot.hooks.raiseHooks();
        sleep(1000);

        moveBackwardByInches(0.5, 2500, 3);

        rotate(88);
    }

    public void runSideRed()
    {
        log("runSideRed");
        sleep(1000);
      //  if(!isStopRequested())
        //moveRedFoundation();

        // wait before parking?

        moveEnc(0.5,10,14);
        moveEnc(0.5,10,6);
        //strafeLeftByInches(0.5, 10000, STEP_5_STRAFE_TO_SIDE_PARK_INCHES);
    }




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

        // fix this
        rotate(88);

        // maybe turn sideways
        robot.hooks.raiseHooks();
        sleep(1000);

        moveBackwardByInches(0.5, 2500, 5);

        rotate(-88);
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

        strafeRightByInches(0.5, 8000, 28);
        moveForwardByInches(0.5, 5000, 38);
        strafeRightByInches(0.5, 5000, 14);
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