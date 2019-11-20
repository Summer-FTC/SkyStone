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
        tf.runOpMode();
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
        }
        else if (position == 2) {
            // get stone in position 2;
        }
        else {
            // get stone in position 3;
        }
    }

    public void dropStone()
    {
        // Drop Stone across the tape
    }
}
