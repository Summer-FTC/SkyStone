package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    VenomRobot robot = new VenomRobot();
    double newPower;


    public void log(String message)
    {
        if (telemetry != null)
        {
            telemetry.addData(message, "");
            telemetry.update();
        }
    }

    public void initialize()
    {
        robot.init(hardwareMap, telemetry, true);

        log("BaseLinearOpMode::initialize complete");
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

    public void moveBackward(double angle, double power, double encoderTicks)
    {
        moveForward(angle, -power, -encoderTicks);

    }

    public void moveBackwardWithEncoders(double power, double encoderTicks)
    {
        moveForwardWithEncoders(power, -encoderTicks);
    }

    // TODO: Add a timeout.
    public void moveForwardWithEncoders(double power, double encoderTicks)
    {
        String direction = (encoderTicks > 0) ? "FORWARD" : "BACKWARD";
        power = Math.abs(power);

        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoders();

        boolean active = true;

        robot.log("Starting loop to move " + direction);

        for (DcMotor m : robot.driveTrain.motors)
        {
            m.setTargetPosition((int)encoderTicks);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (active && opModeIsActive())
        {
            active = false;
            StringBuilder message = new StringBuilder();
            message.append("Moving " + direction + "\n");

            active = true;
            for(DcMotor m : robot.driveTrain.motors)
            {
//                if (m.isBusy())
//                {
//                    active = false;
//                }
                // Sometimes the motor is not busy right at the start, so assume we are
                // busy if we haven't moved half of the encoder ticks.
                // Stop when we get close.
                double ticksRemaining = Math.abs(encoderTicks - m.getCurrentPosition());

                // Sometimes the motor is not busy at the very start.
                // If any motor is not busy and we have covered half of the distance, then
                // we should stop.
                if (!m.isBusy() && ticksRemaining < Math.abs(encoderTicks / 2))
                {
                    active = false;
                }

                double powerToSet = 0;
                if (((encoderTicks < 0) && (m.getCurrentPosition() > encoderTicks)) ||
                        ((encoderTicks > 0) && (m.getCurrentPosition() < encoderTicks))) {


                    // Start slowing down when we are within this distance.
                    int startScalingTicks = 500;
                    if (ticksRemaining > startScalingTicks) {
                        powerToSet = power;
                    } else {
                        powerToSet = Math.max(0.125,
                                power * (ticksRemaining / startScalingTicks));
                    }
                }

                message.append("etting power to " + powerToSet + " since m.getCurrentPosition()=" +
                        m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");

                m.setPower(powerToSet);

            }
            robot.log(message.toString());
        }

        robot.log("Loop is done.");

        // Why reset here?
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    public void moveForward(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();

        boolean active = true;

        robot.log("Starting loop.");

        while (active && opModeIsActive()){
            active = false;
            StringBuilder message = new StringBuilder();

            for(DcMotor m : robot.driveTrain.motors) {
                if (((encoderTicks < 0) && (m.getCurrentPosition() > encoderTicks)) ||
                        ((encoderTicks > 0) && (m.getCurrentPosition() < encoderTicks))) {
                    message.append("Setting power to " + power + " since m.getCurrentPosition()=" + m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");
                    m.setPower(power);

                    active = true;
                 //   sleep(50);

                }
                else {
                    m.setPower(0);
                }

            }
            robot.log(message.toString());
        }

        robot.log("Loop is done.");

        // Why reset here?
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    // Old moveForward method:
    /** public void moveForward(double angle, double power, double encoderTicks)
    {
        telemetry.addData("Encoder value: ", robot.driveTrain.getEncoderTicks() + "");
        robot.driveTrain.resetEncoders();
        robot.driveTrain.run();
        telemetry.addData("Encoder value: ", robot.driveTrain.getEncoderTicks() + "");
        while (robot.driveTrain.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            // change: scale down only last 1000 ticks
            newPower = power * ((encoderTicks - robot.driveTrain.getEncoderTicks())/encoderTicks);
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
     **/

//    public void moveBackward(double angle, double power, double encoderTicks)
//    {
//        robot.driveTrain.resetEncoders();
//        robot.driveTrain.runWithoutEncoders();
//
//        boolean active = true;
//
//        robot.log("Starting loop.");
//
//        while (active && opModeIsActive()){
//            active = false;
//            StringBuilder message = new StringBuilder();
//
//            for(DcMotor m : robot.driveTrain.motors) {
//                if (m.getCurrentPosition() < encoderTicks) {
//                    message.append("Setting power to " + power + " since m.getCurrentPosition()=" + m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");
//                    m.setPower(-power);
//                    sleep(50);
//
//                    active = true;
//                }
//                else {
//                    m.setPower(0);
//                }
//            }
//            robot.log(message.toString());
//
//            try {
//                Thread.sleep(100);
//
//            } catch (Exception e) {
//            }
//        }
//
//        robot.log("Loop is done.");
//
//        // Why reset here?
//        robot.driveTrain.resetEncoders();
//        robot.driveTrain.runWithoutEncoders();
//    }

    /**public void moveBackward(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
        while (robot.driveTrain.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            // try to change to scale down at the end
            newPower = power * ((encoderTicks - robot.driveTrain.getEncoderTicks())/encoderTicks);
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
     **/


    public void strafeRight(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();

        boolean active = true;

        robot.log("Starting loop.");

        while (active && opModeIsActive()){
            active = false;
            StringBuilder message = new StringBuilder();

            // change for strafeRight
            for(DcMotor m : robot.driveTrain.motors) {
                if (m.getCurrentPosition() < encoderTicks) {
                    message.append("Setting power to " + power + " since m.getCurrentPosition()=" + m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");
                    m.setPower(-power);

                    active = true;
                }
                else {
                    m.setPower(0);
                }

            }

            sleep(50);

            robot.log(message.toString());

        }

        robot.log("Loop is done.");

        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    /**
    // pass in current angle as the parameter
    public void strafeRight(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
        while (robot.driveTrain.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            newPower = power * ((encoderTicks - robot.driveTrain.getEncoderTicks())/encoderTicks);
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
     **/

    public void strafeLeft(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();

        boolean active = true;

        robot.log("Starting loop.");

        while (active && opModeIsActive()){
            active = false;
            StringBuilder message = new StringBuilder();

            // change for strafeLeft
            for(DcMotor m : robot.driveTrain.motors) {
                if (m.getCurrentPosition() < encoderTicks) {
                    message.append("Setting power to " + power + " since m.getCurrentPosition()=" + m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");
                    m.setPower(-power);
                    sleep(50);

                    active = true;
                }
                else {
                    m.setPower(0);
                }
            }
            robot.log(message.toString());

            try {
                Thread.sleep(100);

            } catch (Exception e) {
            }
        }

        robot.log("Loop is done.");

        // Why reset here?
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    /**public void strafeLeft(double angle, double power, double encoderTicks)
    {
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
        while (robot.driveTrain.getEncoderTicks() < encoderTicks && opModeIsActive())
        {
            newPower = power * ((encoderTicks - robot.driveTrain.getEncoderTicks())/encoderTicks);
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
     **/

    public void moveToLoadingZone()
    {

    }

    // do we need this
    public void rotate()
    {

    }
}
