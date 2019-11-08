package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DemoLinearOpMode", group = "6209")
public class DemoLinearOpMode extends BaseLinearOpMode
{
    @Override
    public void initialize()
    {
        super.initialize();
    }

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        telemetry.addData("Before moving", "");

//        5000/79
//        moveForwardWithEncoders(0.01, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.015, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.02, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.025, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.05, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.075, 50000, 100); sleep(1000);
//        moveForwardWithEncoders(0.1, 50000, 100); sleep(1000);
//
        rotate(45);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);

        rotate(-45);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);

        rotate(90);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);

        rotate(-90);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);

        rotate(170);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);

        rotate(-170);
        sleep(500);
        rotateToAbsoluteYaw(0);
        sleep(2500);
//
//        strafeRightByInches(0.5, 5000, 2); // 2
//        sleep(2000);
//        strafeRightByInches(0.5, 5000, 6); // 8
//        sleep(2000);
//        strafeRightByInches(0.5, 5000, 16); // 24
//        sleep(2000);
//        strafeRightByInches(0.5, 5000, 24); // 48
//        sleep(2000);

//        moveForwardWithEncoders(0.5, 50000, 4000);
//
//        strafeRightWithEncoders(0.5, 50000, 1000);
//        sleep(4000);
//        strafeRightWithEncoders(0.5, 50000, 2000);
//        sleep(4000);
//        strafeRightWithEncoders(0.5, 50000, 4000);
//        sleep(4000);

//        strafeRightWithEncoders(0.5, 50000, 5000);
//        sleep(4000);
//        strafeLeftWithEncoders(0.5, 50000, 5000);

//
//        // Moves in a square
//        moveForwardWithEncoders(0.5, 5000, 4000);
//        sleep(5000);
//
//        // Turns right
//        rotate(-90);
//
//        sleep(3000);
//
//        strafeRightWithEncoders(0.5, 5000, 5000);
//        sleep(5000);

        //  rotate(90);
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