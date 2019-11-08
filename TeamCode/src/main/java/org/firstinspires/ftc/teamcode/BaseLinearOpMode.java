package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.ElementType;
import java.util.HashMap;
import java.util.Map;

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

    public void initialize(LinearOpMode opmode)
    {
        robot.init(hardwareMap, telemetry, true, opmode);

        log("BaseLinearOpMode::initialize complete");
    }


    private void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR)
    {
        moveWithEncoders(direction, power, msTimeOut,
                encFL, encFR, encBL, encBR, null);
    }

    private double getInchesAvg() {
        double encoders = robot.driveTrain.motorBL.getCurrentPosition() +
                robot.driveTrain.motorFL.getCurrentPosition() +
                robot.driveTrain.motorBR.getCurrentPosition() +
                robot.driveTrain.motorFR.getCurrentPosition();
        double cpi = 1440 / (4 * Math.PI);
        return encoders / cpi;
    }

    public void moveEnc(double power, int timeout, double inches) {
        robot.driveTrain.resetEncoders();

        ElapsedTime time = new ElapsedTime();

        telemetry.addData("init Inches", getInchesAvg());
        telemetry.update();

        time.reset();
        while (Math.abs(getInchesAvg()) < inches && time.seconds() < timeout && opModeIsActive()) {
            robot.setMotorFL(power);
            robot.setMotorBL(power);
            robot.setMotorFR(power);
            robot.setMotorBR(power);

            telemetry.addData("Inches left", inches - getInchesAvg());
        }
        robot.setMotorFL(0);
        robot.setMotorBL(0);
        robot.setMotorFR(0);
        robot.setMotorBR(0);

    }

    // TODO: Add a timeout.
    private void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR,
                                  // null means that we don't stop when the yaw changes.
                                  Double targetYawChange)
    {
        if (targetYawChange != null)
        {
            if ((targetYawChange < -180) || (targetYawChange > 180)) {
                throw new IllegalArgumentException("targetYawChange=" + targetYawChange + " must be between -180 and 180");
            }
        }

        power = Math.abs(power);

        long stopTime = msTimeOut + System.currentTimeMillis();

        Map<DcMotor, Integer> motorToEncoder = new HashMap<>();

        motorToEncoder.put(robot.driveTrain.motorFL, encFL);
        motorToEncoder.put(robot.driveTrain.motorFR, encFR);
        motorToEncoder.put(robot.driveTrain.motorBL, encBL);
        motorToEncoder.put(robot.driveTrain.motorBR, encBR);


        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoders();

        robot.log("Starting loop to move " + direction);

        double initialYaw = robot.imu.getYaw();

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setTargetPosition(motorToEncoder.get(m));
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        boolean active = true;

        while (System.currentTimeMillis() < stopTime && active && opModeIsActive())
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
                        powerToSet = Math.max(0.1,
                                power * (ticksRemaining / startScalingTicks));
                    }
                }

                // If we are rotating, then we can stop the loop once we achieve the desired yaw
                // change.
                if (targetYawChange != null) {
                    message.append("yaw: " + robot.imu.getYaw() + " initYaw: " + initialYaw + "\n");
                    message.append("true diff" + robot.imu.getTrueDiff(initialYaw) + "\n");

                    double currentYawChange = robot.imu.getTrueDiff(initialYaw);
                    if (Math.abs(currentYawChange) >= Math.abs(targetYawChange)) {
                        active = false;
                    }

                    // When targeting values close to 180 or -180, we have to worry about the
                    // difference becoming negative or positive. So if the yaw change is large (>150)
                    // and the signs are different, then we've wrapped around.

                    if ((Math.abs(currentYawChange) > 150) && Math.signum(currentYawChange) != Math.signum(targetYawChange)) {
                        active = false;
                    }

                    // Scale with PID within 30 degrees.
                    double absYawRemaining = Math.abs(targetYawChange) - Math.abs(currentYawChange);
                    double scaleAtYawRemaining = 30;
                    if (absYawRemaining < scaleAtYawRemaining) {
                        double scale = absYawRemaining / scaleAtYawRemaining;
                        powerToSet = Math.max(0.1, scale * power);
                    }
                }

                message.append("Setting power to " + powerToSet + " since m.getCurrentPosition()=" +
                        m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");

                m.setPower(powerToSet);

//                if (yawChange != null) {
//
//                }

//                if ((stopDetector != null) && stopDetector.isDone()) {
//                    active = false;
//                }

            }
            robot.log(message.toString());
        }

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setPower(0);
        }

//        robot.log("Loop is done.");

        // Why reset here?
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    // TODO: Add a timeout.
    public void moveForwardWithEncoders(double power, int msTimeOut, int encoderTicks)
    {
        moveWithEncoders("FORWARD", power, msTimeOut,
                encoderTicks, encoderTicks,
                encoderTicks, encoderTicks);
    }

    public void moveBackwardWithEncoders(double power, int msTimeOut, int encoderTicks)
    {
        moveWithEncoders("BACKWARD", power, msTimeOut,
                -encoderTicks, -encoderTicks,
                -encoderTicks, -encoderTicks);
    }


    public void strafeRightWithEncoders(double power, int msTimeOut, int encoderTicks)
    {
        moveWithEncoders("RIGHT", power, msTimeOut,
                encoderTicks, -encoderTicks,
                -encoderTicks,  encoderTicks);
    }

    public void strafeLeftWithEncoders(double power, int msTimeOut, int encoderTicks)
    {
        moveWithEncoders("LEFT", power, msTimeOut,
                -encoderTicks,  encoderTicks,
                encoderTicks, -encoderTicks);
    }


    public void moveToLoadingZone()
    {

    }

    // do we need this
    public void rotate()
    {

    }

    // positive is left, negative is right
    public void rotate(double degrees)
    {
        // Flip the direction of the encoders if degrees is negative.
        int encoderSign = 1;
        if (degrees < 0) {
            encoderSign = -1;
        }

        int maxEncoder = 5000;
        moveWithEncoders("ROTATE", 0.3, 10000,
                encoderSign * maxEncoder, encoderSign * -maxEncoder,
                encoderSign * maxEncoder, encoderSign * -maxEncoder,
                degrees);

    }

    public void strafeRightByInches(double power, int msTimeOut, double inches)
    {
        int encoderTicks = strafeInchtoEnc(inches);
        strafeRightWithEncoders(power, msTimeOut, encoderTicks);
    }

    public void strafeLeftByInches(double power, int msTimeOut, double inches)
    {
        int encoderTicks = strafeInchtoEnc(inches);
        strafeLeftWithEncoders(power, msTimeOut, encoderTicks);
    }

    public void moveForwardByInches(double power, int msTimeOut, double inches)
    {
        int encoderTicks = moveInchtoEnc(inches);
        moveForwardWithEncoders(power, msTimeOut, encoderTicks);
    }

    public void moveBackwardByInches(double power, int msTimeOut, double inches)
    {
        int encoderTicks = moveInchtoEnc(inches);
        moveBackwardWithEncoders(power, msTimeOut, encoderTicks);
    }

    // left or right
    private int strafeInchtoEnc(double inches)
    {
        return (int)(inches*(4000/78.13));
    }

    // forwards or backwards
    private int moveInchtoEnc(double inches)
    {
        return (int)(inches*(4000/65.8));
    }

    public void displayIMU(int msTimeOut)
    {
        double initialYaw = robot.imu.getYaw();

        long stopTime = msTimeOut + System.currentTimeMillis();

        while (opModeIsActive() && stopTime > System.currentTimeMillis())
        {
            log("YAW : " + robot.imu.getYaw() + "\n" +
                "initialYaw: " + initialYaw + "\n" +
                "true diff: " + robot.imu.getTrueDiff(initialYaw)
//                    "PITCH : " + robot.imu.getPitch() + "\n" +
//                    "ROLL : " + robot.imu.getRoll() + "\n"

            );

        }
    }
}
