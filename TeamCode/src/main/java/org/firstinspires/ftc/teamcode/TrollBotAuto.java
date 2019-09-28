package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class TrollBotAuto extends CustomLinearOpMode
{

    ModernRoboticsI2cRangeSensor rangeSensor;
    boolean SkystoneDetected = false;


    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        try
        {

        } catch (Exception e)
        {
            stop();
        }

        // move to blocks
        moveToDistance(2);

        // Check first block
        turn(-90);
        if(isSkystone())
        {
            driveBackward(2);
        }

        // Check second block
        if (!SkystoneDetected)
            turn(-90);
            if (isSkystone())
            {
                driveBackward(2);
            }
        // Check third block
        // But don't need to because has to be if first two aren't
        if (!SkystoneDetected)
            turn(-90);
            if (isSkystone()) {
                driveBackward(2);
            }
        }

        goForward(12.0);

        public void isSkystone() {
            // if phone detects skystone
                return true;
                return false;
        }

        public void driveForward()
        {

        }

}