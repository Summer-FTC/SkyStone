package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "ParkLinearOpMode" , group = "6209")

public class ParkLinearOpMode extends BaseLinearOpMode
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
        telemetry.addData("Status", "Running Autonomous!");
        telemetry.update();
        initialize();
        waitForStart();

        parkSide();
    }

    public void parkSide() {
        // So we don't get in the other team's way
        //sleep(20000);
        strafeRightByInches(0.5, 3000, 50);
    }

    public void parkCenter() {
        //sleep(15000);
        strafeRightByInches(0.5, 8000, 25);
        moveForwardByInches(0.5, 5000, 22);
        strafeRightByInches(0.5, 5000, 25);
    }
}