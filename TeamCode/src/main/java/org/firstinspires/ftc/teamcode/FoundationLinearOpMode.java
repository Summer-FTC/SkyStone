package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class FoundationLinearOpMode extends BaseLinearOpMode
{

    FoundationHookController found = new FoundationHookController();

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

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

    public void pullFoundation()
    {
        found.extendHooks();
        // hook on and pull back
        found.retractHooks();
    }

    public void park() {
        // park robot: center or side?
    }

}