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

        waitForStart();

        telemetry.addData("Before moving", "");

        moveForwardWithEncoders(0.5, 5000, 1500);
        strafeRightWithEncoders(0.5, 5000, 1500);
        moveBackwardWithEncoders(0.5, 5000, 1500);
        strafeLeftWithEncoders(0.5, 5000, 1500);

        rotate(90);
        sleep(1000);

        rotate(-90);
        sleep(1000);

        rotate(135);
        sleep(1000);

        rotate(-45);
        sleep(1000);

        rotate(180);
        sleep(1000);

        rotate(-180);
        sleep(1000);
//        displayIMU(30000);

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
    }

    public void pullFoundation()
    {
        robot.hooks.lowerHooks(true);
        // pull back
        robot.hooks.lowerHooks(false);
    }

    public void park() {
        // park robot on line: center or side?
    }

}