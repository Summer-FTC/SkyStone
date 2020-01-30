package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;


@Autonomous(name = "DoubleSkystone", group = "6209")
public class DoubleSkystone extends BaseLinearOpMode
{
    boolean configOnly = false;
    boolean isBlue = true;
    boolean parkOnSide = false; // This doesn't do anything. We always park center.


    private static int MOVE_OFF_OF_WALL_DISTANCE = 2;
    private static int STRAFE_TO_POS_1_OR_2_DIST = 4;
    private static int STRAFE_TO_POS_3_DIST = 4;
    private static int MOVE_FORWARD_TO_GET_1ST_STONE_DIST = 27;
    private static int MOVE_BACKWARD_AFTER_GRAB_DIST = 12;
    private static int MOVE_BACKWARD_AFTER_GRAB_BEFORE_TURN_DIST = 6;
    private static int STRAFE_UNDER_BRIDGE_1ST_STONE_DIST = 48;
    private static int STRAFE_BACK_TO_GET_2ND_STONE_DIST = 70;
    private static int MOVE_FORWARD_TO_GET_2ND_STONE_DIST = 16;
    private static int DEPOSIT_UNDER_BRIDGE_2ND_STONE_DIST = 73;
    private static int PARK_DIST = 20;

    int position = 3;

    @Override
    protected boolean usesVuforia() {
        return true;
    }

    @Override
    public void runOpMode()
    {
        initialize(true);
        configMode();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            if (position == 3) {
                driveTo3();
                // Adjust the remaining strafe movements by the width of a block.
                STRAFE_UNDER_BRIDGE_1ST_STONE_DIST += 8;
                STRAFE_BACK_TO_GET_2ND_STONE_DIST += 8;
                DEPOSIT_UNDER_BRIDGE_2ND_STONE_DIST += 8;
            }
            else {
                driveTo1Or2();
            }
            lowerHook();
            deposit123();
            driveTo456();
            lowerHook();

            deposit456WithTurning();
            parkWhenTurned();
        }
    }


    public void lowerHook()
    {
        // Remember R and L are relative to the front of the robot, and we're lowering them from the back.
        if (isBlue) {
            if (position == 1)
                robot.stoneHooks.lowerOneHook("L");
            else
                robot.stoneHooks.lowerOneHook("R");
        } else {
            if (position == 1)
                robot.stoneHooks.lowerOneHook("R");
            else
                robot.stoneHooks.lowerOneHook("L");
        }
    }

    // EVERYTHING IS FLIPPED BECAUSE ROBOT IS BACKWARDS
    public void driveTo1Or2()
    {
        if(isBlue)
        {
            moveBackwardByInches(0.5, MOVE_OFF_OF_WALL_DISTANCE);
            strafeRightByInches(0.85, STRAFE_TO_POS_1_OR_2_DIST);
            moveBackwardByInches(1, MOVE_FORWARD_TO_GET_1ST_STONE_DIST);
        } else {
            moveBackwardByInches(0.5, MOVE_OFF_OF_WALL_DISTANCE);
            strafeLeftByInches(0.85, STRAFE_TO_POS_1_OR_2_DIST);
            moveBackwardByInches(1, MOVE_FORWARD_TO_GET_1ST_STONE_DIST);
        }

    }

    public void driveTo3()
    {
        moveBackwardByInches(0.5, MOVE_OFF_OF_WALL_DISTANCE);

        if(isBlue)
        {
            strafeLeftByInches(0.85, STRAFE_TO_POS_3_DIST);
        } else {
            strafeRightByInches(0.85, STRAFE_TO_POS_3_DIST);
        }
        moveBackwardByInches(1, MOVE_FORWARD_TO_GET_1ST_STONE_DIST);
    }



    public void driveTo456 ()
    {
        if(isBlue) {
            strafeLeftByInches(1, STRAFE_BACK_TO_GET_2ND_STONE_DIST);
        }
        else {
            strafeRightByInches(1, STRAFE_BACK_TO_GET_2ND_STONE_DIST);

        }
        moveBackwardByInches(1, MOVE_FORWARD_TO_GET_2ND_STONE_DIST);

    }

    // EVERYTHING IS FLIPPED BECAUSE ROBOT IS BACKWARDS!
    public void deposit123()
    {
        moveForwardByInches(0.85, MOVE_BACKWARD_AFTER_GRAB_DIST);

        if (isBlue)
            strafeRightByInches(1, STRAFE_UNDER_BRIDGE_1ST_STONE_DIST);
        else
            strafeLeftByInches(1, STRAFE_UNDER_BRIDGE_1ST_STONE_DIST);

        robot.stoneHooks.raiseHooks();
    }


    // Turning gives us more room for error when going under the bridge so that we don't
    // hit the other robot.
    public void deposit456WithTurning()
    {
        moveForwardByInches(0.85, MOVE_BACKWARD_AFTER_GRAB_BEFORE_TURN_DIST);

        // We can't turn if we are on the wall, so we strafe away for a bit.
        double strafeAwayFromWallInches = 6;

        // Ideally the yaws would be 90, but we overshoot it a bit.
        if (isBlue) {
            strafeRightByInches(1, strafeAwayFromWallInches);
            rotate(-87, 0.3);
           // rotateToAbsoluteYaw(-87);
        }
        else {
            strafeLeftByInches(1, strafeAwayFromWallInches);
           // rotateToAbsoluteYaw(87);
            rotate(87, 0.3);
        }

        moveBackwardByInches(1, DEPOSIT_UNDER_BRIDGE_2ND_STONE_DIST - strafeAwayFromWallInches);

        robot.stoneHooks.raiseHooks();

        // We need to sleep for a little longer than in the strafing case because
        // the hook needs to be entirely clear of the stone before moving.
        sleep(500);
    }

    public void parkWhenTurned() {
        moveForwardByInches(.8, PARK_DIST);

        // Move into the bridge, forward is facing the depot end.
        if (isBlue) {
            strafeLeftByInches(1, 6);
        }
        else {
            strafeRightByInches(1, 6);
        }
    }

    public void deposit456WithStrafing()
    {
        moveForwardByInches(0.85, MOVE_BACKWARD_AFTER_GRAB_DIST);

        if (isBlue)
            strafeRightByInches(1, DEPOSIT_UNDER_BRIDGE_2ND_STONE_DIST);
        else
            strafeLeftByInches(1, DEPOSIT_UNDER_BRIDGE_2ND_STONE_DIST);

        robot.stoneHooks.raiseHooks();
    }

    public void parkWithStrafing() {
        if (isBlue) {
            strafeLeftByInches(1, PARK_DIST);
        }
        else {
            strafeRightByInches(1, PARK_DIST);
        }

        // Move into the bridge.
        moveBackwardByInches(0.5, 6);
    }

    public void configMode() {
        String lastModes = "";
        telemetry.addData("Entering " , "ConfigMode");
        telemetry.update();
        do {
            if (VenomUtilities.isValueChangedAndEqualTo("1.y", gamepad1.y, true))
                isBlue = !isBlue;

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
        modes+=", Starting="+(isBlue ?"Blue":"Red");
        modes+=", Starting="+(parkOnSide?"Side":"Center");


        telemetry.addData("Alliance (Y)", isBlue ?"Blue":"Red");

        // We always park center.
//        telemetry.addData("Park Location (A)", parkOnSide?"Side":"Center");

        if (configOnly) telemetry.addData("ConfigMode" , "Press right bumper to leave config mode.");
        if (update) telemetry.update();

        if (!modes.equals(lastModes)) {
            RobotLog.i(modes);
            lastModes=modes;
        }
    }


    @Override
    public synchronized void waitForStart() {
        do {
            // Make sure that we go through this at least once.
            try {
                StonePosition sp = getSkystonePosition();
                position = sp.getPosition(isBlue);
                this.wait(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        } while (!isStarted());
    }

}
