package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DemoLinearOpMode", group = "6209")
public class DemoLinearOpMode extends BaseLinearOpMode
{
    @Override
    public void initialize() {
        super.initialize();
    }

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        telemetry.addData("Before moving", "");

        // Moves in a square
        moveForwardWithEncoders(0.5, 5000, 4000);
        sleep(5000);

        // Turns right
        rotate(-90);

        sleep(3000);

        strafeRightWithEncoders(0.5, 5000, 5000);
        sleep(5000);

//        rotate(90);
//        sleep(1000);
//
//        rotate(-90);
//        sleep(1000);
//
//        rotate(180);
//        sleep(1000);
//
//        rotate(-180);
//        sleep(1000);

        // Lower hooks
        // Raise hooks

        // Raise lift
        // Lower lift

        // Intake in
        // Intake out

    }
}