package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FoundationLinearOpMode", group = "6209")
public class FoundationLinearOpMode extends BaseLinearOpMode
{
    @Override
    public void initialize() {
        super.initialize();
    }

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();
        telemetry.addData("Before moving", "");
        moveForward(0, 0.5, 3000);
        // strafeLeft(0, 0.5, 6000);
        // moveBackward(0, 0.5, 9000);

        // strafeRight(0, 0.5, 12000);
        // 1440 ticks for 100 mm/3.937 inches
        telemetry.addData("After moving", "");
        telemetry.update();
    }

    public void moveToFoundation()
    {
        // if red
        // test # of encoder ticks
            // use tele op to figure out
        // We can also use the distance in inches to find out how far we need to go.
        // I think encoders would be a little inaccurate.
        strafeRight(0, 0.5, 3000);
    }

    public void pullFoundation()
    {
        //found.extendHooks();
        // hook on and pull back
        //found.retractHooks();
    }

    public void park() {
        // park robot on line: center or side?
    }

}