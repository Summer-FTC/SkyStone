package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

    }

    public void moveToFoundation()
    {
        // if red
        // test # of encoder ticks
            // use tele op to figure out
        strafeRight(0, 0.5, 3000);
    }

}