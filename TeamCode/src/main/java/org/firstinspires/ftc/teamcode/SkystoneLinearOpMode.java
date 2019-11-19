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
    protected static final String VUFORIA_KEY = "";
    protected VuforiaLocalizer vuforia;
    protected TFObjectDetector tfod;

    ElapsedTime eTime;
    protected ElapsedTime time = new ElapsedTime();

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        // move forward to be in correct distance of sensing 2 stones
        findSkystone();
        // strafe across line
        dropStone();
        // strafe back
        findSkystone();
        // strafe across line
        dropStone();
        park();
    }

    public void findSkystone() throws InterruptedException {
        // Tensor Flow stuff: detect Skystones

        // look at left 2 stones
        // if Skystone there, returns location and match with either first or second
        // if Skystone not there, go to third location
        // get Skystone
        TensorFlowSkyStone tf = new TensorFlowSkyStone();
        tf.runOpMode();
    }

    public void getStone() throws InterruptedException
    {
        // use output to get stone
    }

    public void dropStone()
    {
        // Drop Stone across the tape
    }

    public void park()
    {
        // park robot on line: center or side?
    }
}
