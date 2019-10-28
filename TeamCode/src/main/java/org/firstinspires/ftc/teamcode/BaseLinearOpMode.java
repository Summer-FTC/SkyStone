package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    VenomRobot robot;

    public void initialize()
    {
        MecanumDriveController drive = new MecanumDriveController("red");
        robot = new VenomRobot(drive);
        robot.init(hardwareMap, telemetry, true);
    }


    // add time out later?
    public double PID(double angle)
    {
        double kP = 0.4/90;
        double minSpeed = 0.1;

            double angleError = robot.imu.getTrueDiff(angle);

            double PIDchange = kP * angleError;

            if (PIDchange > 0 && PIDchange < minSpeed)
                PIDchange = minSpeed;
            else if (PIDchange < 0 && PIDchange > -minSpeed)
                PIDchange = -minSpeed;

        return PIDchange;
    }


    public void moveForward(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        while (robot.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            double newPower = power * ((encoderTicks - robot.getEncoderTicks())/encoderTicks);
            double PIDchange = PID(angle);

            if (newPower < 0.1)
            {
                newPower = 0.1;
            }

            // TODO: figure out + and -
            // either set left or right to negative
            // if statement for if knocked left or right
            // curves left
            robot.setMotorBL(PIDchange + newPower);
            robot.setMotorBR(-PIDchange + newPower);
            robot.setMotorFL(PIDchange + newPower);
            robot.setMotorFR(-PIDchange + newPower);
        }
    }

    public void moveBackward(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        while (robot.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            // try to change to scale down at the end
            double newPower = power * ((encoderTicks - robot.getEncoderTicks())/encoderTicks);
            double PIDchange = PID(angle);

            if (newPower < 0.1)
            {
                newPower = 0.1;
            }

            // TODO: figure out + and -
            robot.setMotorBL(PIDchange - newPower);
            robot.setMotorBR(-PIDchange - newPower);
            robot.setMotorFL(PIDchange - newPower);
            robot.setMotorFR(-PIDchange - newPower);
        }
    }

    // pass in current angle as the parameter
    public void strafeRight(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        while (robot.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            double newPower = power * ((encoderTicks - robot.getEncoderTicks())/encoderTicks);
            double PIDchange = PID(angle);

            if (newPower < 0.1)
            {
                newPower = 0.1;
            }

            // TODO: figure out + and -
            // Test
            robot.setMotorBL(PIDchange - newPower);
            robot.setMotorBR(-PIDchange + newPower);
            robot.setMotorFL(PIDchange + newPower);
            robot.setMotorFR(-PIDchange - newPower);

        }
    }

    public void strafeLeft(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        while (robot.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            double newPower = power * ((encoderTicks - robot.getEncoderTicks())/encoderTicks);
            double PIDchange = PID(angle);

            if (newPower < 0.1)
            {
                newPower = 0.1;
            }

            // TODO: figure out + and -
            robot.setMotorBL(PIDchange + newPower);
            robot.setMotorBR(-PIDchange - newPower);
            robot.setMotorFL(PIDchange - newPower);
            robot.setMotorFR(-PIDchange + newPower);
        }
    }

    public void moveToLoadingZone()
    {

    }

    // do we need this
    public void rotate()
    {

    }
}
