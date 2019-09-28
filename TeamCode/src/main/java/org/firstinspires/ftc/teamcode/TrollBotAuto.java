package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

public class TrollBotAuto extends CustomLinearOpMode
{
    boolean SkystoneDetected = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        try {
        } catch (Exception e) {
        }

        // move to blocks
        // driveForward();

        // Check first block
        if (isSkystone())
            driveBackward();

        // Check second block
        if (!SkystoneDetected) {
            turnRight();
            if (isSkystone())
                driveBackward();
        }

        // Check third block
        // But don't need to because has to be if first two aren't
        if (!SkystoneDetected)
            turnRight();
            if (isSkystone()) {
                driveBackward();
            }
        }

        public boolean isSkystone() {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals("Skystone")) {
                    SkystoneDetected = true;
                    return true;
                }
            }
            SkystoneDetected = false;
            return false;
        }

        public void driveBackward()
        {
            motorBL.setPower(-0.25);
            motorFL.setPower(-0.25);
            motorBR.setPower(-0.25);
            motorFR.setPower(-0.25);
            sleep(4000);
        }

        public void turnLeft() {
            motorBL.setPower(-0.25);
            motorFL.setPower(-0.25);
            motorBR.setPower(0.25);
            motorFR.setPower(0.25);
            sleep(500);
        }

        public void turnRight() {
            motorBL.setPower(0.25);
            motorFL.setPower(0.25);
            motorBR.setPower(-0.25);
            motorFR.setPower(-0.25);
            sleep(500);
        }
}