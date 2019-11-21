package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous
@Disabled
public class SkystoneLinearOpMode extends BaseLinearOpMode
{
    TensorFlowSkyStone tf;
    int skystonePosition;


    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void initialize() {
        super.initialize();
        tf = new TensorFlowSkyStone();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // move forward first
        moveForwardByInches(0.5, 20);
        tf.runOpMode();
        // wait for phone to find position
        sleep(3000);
        skystonePosition = tf.getSkystonePosition();
        tf.stop();

        // move to skystonePosition

        getStone(skystonePosition);

        // move across line to foundation
        dropStone(); // on foundation

        // get in the right position
        robot.hooks.lowerHooks();
        // move and rotate
        robot.hooks.raiseHooks();

        // go park
    }

    public void getStone(int position) throws InterruptedException
    {
        if (position == 1) {
            // get stone in position 1;
            strafeRightByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
        }
        else if (position == 2) {
            // get stone in position 2;
            strafeLeftByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
        }
        else {
            // get stone in position 3;
            strafeLeftByInches(0.5, 10);
            moveForwardByInches(0.5, 5);
        }
    }

    public void dropStone()
    {
        // Drop Stone across the tape
    }
}
