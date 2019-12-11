package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous

public class SkystoneLinearOpMode extends BaseLinearOpMode
{
    TensorFlowSkyStone tf;
    int skystonePosition;


    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void initialize()
    {
        super.initialize();
        tf = new TensorFlowSkyStone();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // move forward first
     //   moveForwardByInches(0.5, 20);

        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();
        initialize();
        waitForStart();

        tf.runOpMode();
      //   wait for phone to find position
        sleep(3000);

        skystonePosition = tf.getSkystonePosition();
        tf.stop();

        robot.hooks.lowerHooks();

        // move to skystonePosition
       // moveToStone(skystonePosition);

        //        // move across line to foundation
//        dropStone(); // on foundation
//
//        // get in the right position
//        robot.hooks.lowerHooks();
//        // move and rotate
//        robot.hooks.raiseHooks();

        // go park
    }


    public void pickBetween2()
    {

    }

    // TO DO change positions 1 2 3 to constants
    public void moveToStone(int position) throws InterruptedException
    {
        if (position == 1)
        {
            // get stone in position 1;
            strafeRightByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
            getStone();
        }

        else if (position == 2)
        {
            // get stone in position 2;
            strafeLeftByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
            getStone();
        }
        else
        {
            // get stone in position 3;
            strafeLeftByInches(0.5, 10);
            moveForwardByInches(0.5, 5);
            getStone();
        }
    }

    public void getStone()
    {
        robot.output.moveClampOutOfRobot();
        robot.output.openClamp();

        moveForwardByInches(0.5, 5);
        robot.output.closeClamp();

        robot.output.startMoveLiftUp();
        sleep(1000);

        moveBackwardByInches(0.5, 5);
    }

    public void dropStone()
    {
        robot.output.startMoveLiftDown();
        sleep(1000);
        robot.output.openClamp();
    }
}
