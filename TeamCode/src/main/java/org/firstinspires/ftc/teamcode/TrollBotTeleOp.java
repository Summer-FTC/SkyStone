package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TrollbotTeleOp", group="TrollBot")


public class TrollBotTeleOp extends CustomOpMode
{
    public void init()
    {
        initialize();
    }

    public void loop()
    {
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
            stopDriveMotors();
        }
    }
}