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

//        moveForwardWithEncoders(0.01, 100); sleep(1000);
//        moveForwardWithEncoders(0.015, 100); sleep(1000);
//        moveForwardWithEncoders(0.02, 100); sleep(1000);
//        moveForwardWithEncoders(0.025, 100); sleep(1000);
//        moveForwardWithEncoders(0.05, 100); sleep(1000);
//        moveForwardWithEncoders(0.075, 100); sleep(1000);
//        moveForwardWithEncoders(0.1, 100); sleep(1000);
//

        strafeLeftByInches(0.5, 20);
        moveBackwardByInches(0.5, 20);
        strafeRightByInches(0.5, 20);
        moveForwardByInches(0.5, 20);

        strafeLeftByInches(0.5, 20);
        moveBackwardByInches(0.5, 20);
        strafeRightByInches(0.5, 20);
        moveForwardByInches(0.5, 20);

        //
//        rotateToAbsoluteYaw(45);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);
//
//        rotateToAbsoluteYaw(-45);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);
//
//        rotateToAbsoluteYaw(90);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);
//
//        rotateToAbsoluteYaw(-90);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);
//
//        rotateToAbsoluteYaw(170);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);
//
//        rotateToAbsoluteYaw(-170);
//        sleep(500);
//        rotateToAbsoluteYaw(0);
//        sleep(2500);

//        strafeRightByInches(0.5, 2); // 2 , 24
//        sleep(2000);
//        strafeRightByInches(0.5, 6); // 8 , 32
//        sleep(2000);
//        strafeRightByInches(0.5, 16); // 24 , 48
//        sleep(2000);
//        strafeRightByInches(0.5, 24); // 48 , 72
//        sleep(2000);

//        moveForwardWithEncoders(0.5, 4000);
//
//        strafeRightWithEncoders(0.5, 1000);
//        sleep(4000);
//        strafeRightWithEncoders(0.5, 2000);
//        sleep(4000);
//        strafeRightWithEncoders(0.5, 4000);
//        sleep(4000);

//        strafeRightWithEncoders(0.5, 5000);
//        sleep(4000);
//        strafeLeftWithEncoders(0.5, 5000);

//
//        // Moves in a square
//        moveForwardWithEncoders(0.5, 4000);
//        sleep(5000);
//
//        // Turns right
//        rotate(-90);
//
//        sleep(3000);
//
//        strafeRightWithEncoders(0.5, 5000);
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