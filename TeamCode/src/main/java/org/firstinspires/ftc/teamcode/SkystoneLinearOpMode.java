package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous
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
        // move to stones
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

        // use intake to get Skystone
        // scans from middle --> right
        // if !Skystone
        // keep strafing
        // if Skystone
        // get Skystone

        // Scan first stones from left to right until first Skystone detected
        // If in first three, come back for second Skystone
        // Scan last three stones
        // If second Skystone still there, get
        // If in last three, ally already got first Skystone, get second
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
