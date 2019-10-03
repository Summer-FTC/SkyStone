package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;

@Autonomous (name = "TrollBotAuto", group = "6209")

public class TrollBotAuto extends CustomLinearOpMode
{
    /*
    boolean SkystoneDetected = false;


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        initialize();

            double yL = -gamepad1.left_stick_y;
            double yR = -gamepad1.right_stick_y;
            double motorScale = 1;

            if (Math.abs(yL) > 0.2 || Math.abs(yR) > 0.2)
            {
                motorFL.setPower(leftABSMotorVal(yL) * motorScale);
                motorFR.setPower(rightABSMotorVal(yR) * motorScale);
            }

            else
            {
                motorFL.setPower(0);
                motorFR.setPower(0);
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
          //  motorBL.setPower(-0.25);
            motorFL.setPower(-0.25);
          //  motorBR.setPower(-0.25);
            motorFR.setPower(-0.25);
            sleep(4000);
        }

        public void turnLeft() {
         //   motorBL.setPower(-0.25);
            motorFL.setPower(-0.25);
         //   motorBR.setPower(0.25);
            motorFR.setPower(0.25);
            sleep(500);
        }

        public void turnRight() {
         //   motorBL.setPower(0.25);
            motorFL.setPower(0.25);
         //   motorBR.setPower(-0.25);
            motorFR.setPower(-0.25);
            sleep(500);
        }

    // Ignore this. I copied and pasted it from CustomOpMode because this is technically an autonomous class
    public double leftABSMotorVal(double joyStickVal)
    {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorFL.getPower() + maxJump)
        {
            return Range.clip(motorFL.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorFL.getPower() - maxJump)
        {
            return Range.clip(motorFL.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }

    // Also ignore this
    public double rightABSMotorVal(double joyStickVal) {
        double maxJump = .4;
        double c = .1;
        if (joyStickVal >= motorFR.getPower() + maxJump) {
            return Range.clip(motorFR.getPower() + c, -1, joyStickVal);
        }
        else if (joyStickVal < motorFR.getPower() - maxJump) {
            return Range.clip(motorFR.getPower() - c, joyStickVal, 1);
        }
        else return joyStickVal;
    }

     */
}