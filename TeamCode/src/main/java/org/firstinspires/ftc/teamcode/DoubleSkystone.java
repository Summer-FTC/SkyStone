package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;


@Autonomous(name = "DoubleSkystone", group = "6209")
public class DoubleSkystone extends BaseLinearOpMode
{

        boolean configOnly = false;
        boolean isStartingBlue = true;
        boolean parkOnSide = true;

        // TODO: test these distances
        // TODO: change this to work with the new mechanism

        @Override
        public void runOpMode()
        {
            super.initialize(true);
            configMode();

            telemetry.addData(">", "Press Play to start op mode");
            telemetry.update();
            waitForStart();

            if (opModeIsActive())
            {
                moveForwardByInches(0.6, 14);

                // IF IT'S THE FIRST STONE
                if (isSkystone(isStartingBlue)) {
                    moveClampOutInAuto();
                    grabAndTurn(isStartingBlue);
                    moveForwardByInches(1, 30);
                    dropStone();

                    // return to get the 4th stone
                    getNextStone(isStartingBlue, 4);
                    moveForwardByInches(1, 54);
                    dropStone();

                    parkOnlySkystone();
                }

                else
                {
                    // goes to the next stone
                    strafeToStone(isStartingBlue);

                    // IF IT'S THE SECOND STONE
                    if(isSkystone(isStartingBlue)) {
                        moveClampOutInAuto();
                        grabAndTurn(isStartingBlue);
                        moveForwardByInches(1, 38);
                        dropStone();

                        getNextStone(isStartingBlue, 5);
                        moveForwardByInches(1, 62);
                        dropStone();

                        parkOnlySkystone();
                    }
                    else
                    {
                        // goes to the next stone
                        strafeToStone(isStartingBlue);
                        moveClampOutInAuto();
                        grabAndTurn(isStartingBlue);
                        moveForwardByInches(1, 46);

                        // Can we get the 6th stone???
                        getNextStone(isStartingBlue, 6);
                        moveForwardByInches(1, 70);
                        dropStone();

                        parkOnlySkystone();
                    }
                }
            }
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

        public void parkOnlySkystone() {

            if (isStartingBlue) {
                strafeRightByInches(0.7, 3);
            } else {
                strafeLeftByInches(0.7, 3);
            }
            moveBackwardByInches(0.7, 20);
            /**if (parkOnSide) {
             moveBackwardByInches(0.7, 20);
             } else {
             moveBackwardByInches(0.7, 20);
             }**/
        }


        // TODO: do we need to hit the wall to reset ourselves?
    public void getNextStone(boolean blue, int numStone) {
        moveBackwardByInches(0.75, 12);
        if (blue) {
            // moves backwards before it turns so the clamp doesn't hit the sky bridge
            // turn counter clockwise 90 (back to starting yaw)
            rotateToAbsoluteYaw(0);

            // TODO: FIGURE OUT THESES DISTANCES!!!
            if (numStone == 4)
                strafeRightByInches(1, 26);

            // gets the 5th either way bc we can't get the 6th i don't think
            else
                strafeRightByInches(1, 32);

            grabAndTurn(true);

        } else {
            // turn clockwise 90 (back to starting yaw)
            rotateToAbsoluteYaw(0);
            // strafe to correct stone
            if (numStone == 4)
                strafeLeftByInches(1, 26);

            // gets the 5th either way bc we can't get the 6th i don't think
            else
                strafeLeftByInches(1, 32);

            grabAndTurn(false);
        }
    }

}
