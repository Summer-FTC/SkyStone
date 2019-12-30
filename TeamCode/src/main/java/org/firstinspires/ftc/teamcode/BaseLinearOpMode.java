package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.HashMap;
import java.util.Map;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    // This power will move the robot slowly. It actually moves at .015 (very slowly).
    private static double MIN_POWER = 0.10;
    private static double RAMP_UP_MS = 500;
    private static double RAMP_DOWN_MS = 1000;

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


    private void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR)
    {
        moveWithEncoders(direction, power, msTimeOut,
                encFL, encFR, encBL, encBR, null);
    }


    private void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR,
                                  // null means that we don't stop when the yaw changes.
                                  Double targetYawChange)
    {
        // Make sure the timeout is big enough.
        // int maxEnc = Math.max(Math.max(Math.abs(encFL), encFR), Math.max(encBL, encBR));
        // msTimeOut = Math.max(msTimeOut, (int)Math.ceil(maxEnc / power));

        // TODO: Take this out.
        msTimeOut = Math.max(10000, msTimeOut);

        if (targetYawChange != null)
        {
            if ((targetYawChange < -180) || (targetYawChange > 180)) {
                throw new IllegalArgumentException("targetYawChange=" + targetYawChange + " must be between -180 and 180");
            }
        }

        power = Math.abs(power);

        long stopTime = msTimeOut + System.currentTimeMillis();
        long startTime = System.currentTimeMillis();

        Map<DcMotor, Integer> motorToEncoder = new HashMap<>();
        Map<DcMotor, Double> motorToPowerRatio = new HashMap<>();

        motorToEncoder.put(robot.driveTrain.motorFL, encFL);
        motorToEncoder.put(robot.driveTrain.motorFR, encFR);
        motorToEncoder.put(robot.driveTrain.motorBL, encBL);
        motorToEncoder.put(robot.driveTrain.motorBR, encBR);

        int maxEnc = 0;
        for (Integer enc : motorToEncoder.values())
        {
            maxEnc = Math.max(Math.abs(enc), maxEnc);
        }


        robot.driveTrain.resetEncoders();
        robot.driveTrain.runUsingEncoders();


        robot.log("Starting loop to move " + direction);

        double initialYaw = robot.imu.getYaw();

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setTargetPosition(motorToEncoder.get(m));
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Scale each motor power relative to encoder ticks
            motorToPowerRatio.put(m, ((double)motorToEncoder.get(m)/maxEnc));

            m.setPower(MIN_POWER);
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
                    // Accelerate slowly at the start.
                    double rampUpMaxPower = MIN_POWER + (System.currentTimeMillis()-startTime) / RAMP_UP_MS;

                    // Figuring out how to ramp down is more complicated.

                    // At a ramp of 500ms, we got to max power in 12 inches.
                    // distance = .5 * a * t^2
                    // Start ramping down from full power at 12 inches.
                    // 12 inches = .5 * a * .5^2
                    // a = 12 inches / .5^3.
                    // a = 96 inches per second per second.

                    double inchesToEnd  = ticksRemaining * (1.0 / moveInchtoEnc(1));
                    double rampDownAcclerationInchesPerSecond2 = 96;
                    double secondsToEnd  = Math.sqrt(2.0 * inchesToEnd / rampDownAcclerationInchesPerSecond2); // d = .5 a * t^2
                    double msToEnd = 1000 * secondsToEnd;
                    double rampDownMaxPower = MIN_POWER + (msToEnd) / RAMP_DOWN_MS;

                    powerToSet = Math.min(power,
                            Math.min(rampUpMaxPower, rampDownMaxPower));

                    message.append("inchesToEnd: " + inchesToEnd + "\n");
                    message.append("msToEnd: " + msToEnd + "\n");
                    message.append("rampDownMaxPower: " + rampDownMaxPower + "\n");
                    message.append("rampUpMaxPower: " + rampUpMaxPower + "\n");
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
                        powerToSet = Math.max(MIN_POWER, scale * power);
                    }
                }

                message.append("Setting power to " + powerToSet + " since m.getCurrentPosition()=" +
                        m.getCurrentPosition() + " encoderTicks=" + encoderTicks + "\n");

                m.setPower(powerToSet * motorToPowerRatio.get(m));
            }
            robot.log(message.toString());
            idle();
        }

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setPower(0);
        }

        // Why reset here?
        robot.driveTrain.resetEncoders();
        robot.driveTrain.runWithoutEncoders();
    }

    public void moveForwardWithEncoders(double power, int encoderTicks)
    {
        int msTimeOut = moveTimeoutFromTicks(power, encoderTicks);
        moveWithEncoders("FORWARD", power, msTimeOut,
                encoderTicks, encoderTicks,
                encoderTicks, encoderTicks);
    }

    public void moveBackwardWithEncoders(double power, int encoderTicks)
    {

        log("Moving BACKWARD by " + encoderTicks);


        int msTimeOut = moveTimeoutFromTicks(power, encoderTicks);
        moveWithEncoders("BACKWARD", power, msTimeOut,
                -encoderTicks, -encoderTicks,
                -encoderTicks, -encoderTicks);
    }


    public void strafeRightWithEncoders(double power, int encoderTicks)
    {
        int msTimeOut = strafeTimeoutFromTicks(power, encoderTicks);
        moveWithEncoders("RIGHT", power, msTimeOut,
                encoderTicks, -encoderTicks,
                -encoderTicks,  encoderTicks);
    }

    public void strafeLeftWithEncoders(double power, int encoderTicks)
    {
        int msTimeOut = strafeTimeoutFromTicks(power, encoderTicks);
        moveWithEncoders("LEFT", power, msTimeOut,
                -encoderTicks,  encoderTicks,
                encoderTicks, -encoderTicks);
    }


    // This is useful to move to an absolute position based on where you started.
    public void rotateToAbsoluteYaw(double yawDegrees)
    {
        double targetYawChange = robot.imu.getTrueDiff(yawDegrees);
        rotate(targetYawChange);

        double actualYaw = robot.imu.getYaw();
        log("Targeting yaw=" + yawDegrees + " actual yaw=" + actualYaw);
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

    private int strafeTimeoutFromTicks(double power, int encoderTicks)
    {
        return moveTimeoutFromTicks(power, encoderTicks);
    }

    private int moveTimeoutFromTicks(double power, int encoderTicks)
    {
        // Derive the timeout from the encoder ticks.
        // Power 1 : 3000 ticks / second.

        double ticksPerSecondAtMaxPower = 3000.0;
        int msInSeconds = 1000;
        int safetyFactor = 2;
        double adjustedPower = Math.max(power, .1);

        double seconds = encoderTicks / (adjustedPower * ticksPerSecondAtMaxPower);
        int timeout = (int)(safetyFactor * msInSeconds * seconds);

        return timeout;
    }

    public void strafeRightByInches(double power, double inches)
    {
        int encoderTicks = strafeInchtoEnc(inches);
        strafeRightWithEncoders(power, encoderTicks);
    }

    public void strafeLeftByInches(double power, double inches)
    {
        int encoderTicks = strafeInchtoEnc(inches);
        strafeLeftWithEncoders(power, encoderTicks);
    }

    public void moveForwardByInches(double power, double inches)
    {
        int encoderTicks = moveInchtoEnc(inches);
        moveForwardWithEncoders(power, encoderTicks);
    }

    public void moveBackwardByInches(double power, double inches)
    {
        int encoderTicks = moveInchtoEnc(inches);
        moveBackwardWithEncoders(power, encoderTicks);
    }

    // left or right
    private int strafeInchtoEnc(double inches)
    {
        return (int)(inches*(5000/71.5));
        // return (int)(inches*(5000/79));
    }

    // forwards or backwards
    private int moveInchtoEnc(double inches)
    {
//        return (int)(inches*(4000/65.8));
        return (int)(inches*(5000/81));
    }

    public void arcBackwardsToAbsoluteYaw(double power, double turningRadius,
                                          double endingAbsoluteYaw, boolean clockwise)
    {
        turningRadius = Math.abs(turningRadius);
        // turningRadius is for the inner wheel
        // Todo: measure btwn wheels (15 for now)
        final double DIST_BTWN_WHEELS = 15;
        final int MAX_ENCODER_TICKS = -20_000;
        double innerToOuterRatio = turningRadius / (turningRadius + DIST_BTWN_WHEELS);
        String direction;
        int encL;
        int encR;
        if(clockwise){
            direction = "Arcing backwards clockwise";
            encL = (int)(MAX_ENCODER_TICKS * innerToOuterRatio);
            encR = MAX_ENCODER_TICKS;
        }
        else{
            direction = "Arcing backwards counterclockwise";
            encL = MAX_ENCODER_TICKS;
            encR = (int)(MAX_ENCODER_TICKS * innerToOuterRatio);
        }

        double targetYawChange = robot.imu.getTrueDiff(endingAbsoluteYaw);
        moveWithEncoders(direction, power, 30_000, encL, encR, encL, encR, targetYawChange);
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
                // "PITCH : " + robot.imu.getPitch() + "\n" +
                // "ROLL : " + robot.imu.getRoll() + "\n"

            );
        }
    }


    public void grabStoneInAuto()
    {
        robot.output.moveClampOutOfRobot();

        robot.output.openClampFully();

        moveForwardByInches(.65, 12);

        robot.output.moveElbowToPosition(robot.output.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);

        robot.output.closeClampFully();

        robot.output.moveElbowToPosition(robot.output.ELBOW_POSITION_OUTSIDE_ROBOT_AND_PARTIALLY_UP);

        robot.output.startMoveLiftUp();
        sleep(200);

        robot.output.stopLift();

    }


    public void dropStone()
    {
        // TODO: Implement me.
//        robot.output.startMoveLiftDown();
//        sleep(1000);
//        robot.output.startOpeningClamp();
    }
}
