package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Full Run No PID" , group = "6209")
public class AutoTestNoPID extends AutoTest {
    @Override
    protected void setHeadingToHold(Double headingToHold) {
        // Ignore this so that we can compare PID and non PID.
    }
}
