package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

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

    // TODO: Add a timeout.
    public void moveWithEncoders( String direction, double power, int encFL, int encFR, int encBL, int encBR)
    {
        power = Math.abs(power);

        Map<DcMotor, Integer> motorToEncoder = new HashMap<>();

        motorToEncoder.put(robot.driveTrain.motorFL, encFL);
        motorToEncoder.put(robot.driveTrain.motorFR, encFR);
        motorToEncoder.put(robot.driveTrain.motorBL, encBL);
        motorToEncoder.put(robot.driveTrain.motorBR, encBR);


        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoders();

        boolean active = true;

        robot.log("Starting loop to move " + direction);

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setTargetPosition(motorToEncoder.get(m));
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (active && opModeIsActive())
        {
            StringBuilder message = new StringBuilder();
            message.append("Moving " + direction + "\n");

            active = true;
            for(DcMotor m : motorToEncoder.keySet())
            {
                int encoderTicks = motorToEncoder.get(m);

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

    public void strafeRightWithEncoders(double power, int encoderTicks)
    {
        moveWithEncoders("RIGHT", power, encoderTicks, -encoderTicks,
                                                 -encoderTicks,  encoderTicks);
    }

    public void strafeRightTimeBased(double power, double ms)
    {
        long start = System.currentTimeMillis();

        for (DcMotor m : robot.driveTrain.motors)
        {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        while (opModeIsActive() && (ms > Math.abs((System.currentTimeMillis() - start))))
        {
            robot.driveTrain.motorFL.setPower(power);
            robot.driveTrain.motorFR.setPower(-power);
            robot.driveTrain.motorBL.setPower(-power);
            robot.driveTrain.motorBR.setPower(power);
        }

        for (DcMotor m : robot.driveTrain.motors)
        {
            m.setPower(0);
        }
    }

//    public void move




    public void moveToLoadingZone()
    {

    }

    // do we need this
    public void rotate()
    {

    }
}
