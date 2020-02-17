package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";


    private static final String VUFORIA_KEY = "AdlGeb7/////AAABmSRxygbWhU7JmYOsyl0bcLgAAIlM4tFb63K1a+swM0Z2qYggVDcwj/RDVakun/FOpm14tLjtU7UAmIRuCfA1Sah8YloIcX8O6+8Uj/BI3J9D/C3uiTBXzfLEA8Ml4c53WhR2GhQ1LJtfrZ1bjsmY5qksP7i0eaXfFUZ6s1elxJua5gVx4jbuVrh09yaGCfZ3GxymbY3S5ZJWDWiEB7RY5JIHGb01Ar30tzki47QL1YQKHkqM2u1Zm4aJl6/KedqOTc1EL3DNXAYb/jCj/Xnl2pzV7vAKUvhscgHA1MMHo5yPjL2mG6ySKKZnMr0tyjgwAYsyWA5syAi1Bgb+lqeUlsAaD3rssPZfPE0BzXV9dqzG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * hello my friend
     */


    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // This power will move the robot slowly.
    private static double MIN_POWER = 0.10;
    private static double RAMP_UP_MS = 750;
    private static double RAMP_DOWN_MS = 1500;

    // Bitmap dimension constants.
    // THE IMAGE THAT IS CAPTURED IS UPSIDE DOWN!!!
    static int MIDDLE_OF_BLOCK_TO_TOP_OF_IMAGE = 273;
    static int MIDDLE_OF_LEFT_BLOCK_TO_LEFT_EDGE = 940;
    static int MIDDLE_OF_MIDDLE_BLOCK_TO_LEFT_EDGE = 671;
    static int MIDDLE_OF_RIGHT_BLOCK_TO_LEFT_EDGE = 390;

    // If the image weren't upside down.
//    static int MIDDLE_OF_BLOCK_TO_TOP_OF_IMAGE = 442;
//    static int MIDDLE_OF_LEFT_BLOCK_TO_LEFT_EDGE = 350;
//    static int MIDDLE_OF_MIDDLE_BLOCK_TO_LEFT_EDGE = 625;
//    static int MIDDLE_OF_RIGHT_BLOCK_TO_LEFT_EDGE = 900;

    private static int HEIGHT_OF_RECTANGLE = 30;
    private static int WIDTH_OF_RECTANGLE = 60;

    // When we move, we use PID to hold our heading.
    // This starts off to be 0 (see init), which is how the robot is oriented at the start.
    // We update this each time that we rotate to an absolute yaw.
    private Double headingToHold = null;

    VenomRobot robot = new VenomRobot();
    double newPower;


    public void log(String message)
    {
        if (telemetry != null)
        {
            telemetry.addLine(message);
            telemetry.update();
        }
    }

    // We have the isAutonomous option to simulate teleop initialization in AutoTest.
    public void initialize(boolean isAutonomous)
    {
        robot.init(hardwareMap, telemetry, isAutonomous);

        if (isAutonomous) {
            if (usesVuforia() || usesTensorFlow()) {
                initVuforia();
            }

            if (usesTensorFlow()) {
                initTfod();
            }
        }
        // Use setHeadingToHold since AutoTestNoPID overrides it to not save the heading.
        setHeadingToHold(0.0);

        log("BaseLinearOpMode::initialize complete");
    }

    // Subclasses override this to return true if they use Vuforia.
    protected boolean usesVuforia() {
        return false;
    }

    // Subclasses override this to return true if they use Tensor Flow.
    protected boolean usesTensorFlow() {
        return false;
    }

    public void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR)
    {
        moveWithEncoders(direction, power, msTimeOut,
                encFL, encFR, encBL, encBR, null);
    }

    protected void setHeadingToHold(Double headingToHold) {
        this.headingToHold = headingToHold;
    }

    public void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR,
                                  // null means that we don't stop when the yaw changes.
                                  Double targetYawChange)
    {
        // Make this a parameter so that we don't do it all the time? We could even set it as a
        // member variable to set it only for the next iteratin?
        boolean holdHeading = (targetYawChange == null) && (headingToHold != null);

        // These were tuned with https://robotics.stackexchange.com/questions/167/what-are-good-strategies-for-tuning-pid-loops
        // The error is in degrees, but the adjustments are in power changes, so we expect these values to
        // be very small.
        final double PID_KP = 0.04; // Every degree that we are off, we will change the power by this amount.
        final double PID_KD = 0.02;
        final double PID_KI = 0.001;

        // This is the most that we will adjust our power by.
        final double maxHoldHeadingPowerAdjustment = power / 5;

        // This is the maximum that we adjust the power by in a single iteration.
        final double maxAdjustmentPerIteration = PID_KP * 2;

        // We store all of the PID stuff in a csv file so we can analyze it if we want to afterwards.
        StringBuilder csvBuffer = new StringBuilder();
        csvBuffer.append("direction," + direction + "\n");
        csvBuffer.append("power," + power + "\n");
        csvBuffer.append("PID_KP," + PID_KP + "\n");
        csvBuffer.append("PID_KD," + PID_KD + "\n");
        csvBuffer.append("PID_KI," + PID_KI + "\n");
        csvBuffer.append("maxHoldHeadingPowerAdjustment," + maxHoldHeadingPowerAdjustment + "\n");
        csvBuffer.append("maxAdjustmentPerIteration," + maxAdjustmentPerIteration + "\n");
        csvBuffer.append("Sec,yawError,prevYawError,errorDelta,adjust,rawAdjust,prevAdjust,adjDelta,yawErrorIntegral");
        csvBuffer.append(",powFL,powFR,powBL,powBR,velFL,velFR,velBL,velBR");

        // Make sure the timeout is big enough.
        // int maxEnc = Math.max(Math.max(Math.abs(encFL), encFR), Math.max(encBL, encBR));
        // msTimeOut = Math.max(msTimeOut, (int)Math.ceil(maxEnc / power));

        long methodStartMillis = System.currentTimeMillis();

        // TODO: Take this out.
        msTimeOut = Math.max(10000, msTimeOut);

        if (targetYawChange != null)
        {
            if ((targetYawChange < -180) || (targetYawChange > 180)) {
                throw new IllegalArgumentException("targetYawChange=" + targetYawChange +
                        " must be between -180 and 180");
            }
        }

        power = Math.abs(power);

        long stopTime = msTimeOut + System.currentTimeMillis();

        Map<DcMotor, Integer> motorToEncoder = new LinkedHashMap<>();
        Map<DcMotor, Integer> motorToPosition = new LinkedHashMap<>();
        Map<DcMotor, Double> motorToPowerRatio = new LinkedHashMap<>();

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

        robot.log("Starting loop to move " + direction);

        final double initialYaw = robot.imu.getYaw();

        // The power that we want to set for each motor. This is updated each time through the loop.
        Map<DcMotor,Double> motorToDesiredPower = new HashMap<>();

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setTargetPosition(motorToEncoder.get(m));
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Scale each motor power relative to encoder ticks
            motorToPowerRatio.put(m, ((double)motorToEncoder.get(m)/maxEnc));
        }
        motorToPowerRatio = Collections.unmodifiableMap(motorToPowerRatio);

        // Set the motors altogether here rather than in the loop above
        // so that we are starting them at close to the same time.
        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setPower(Math.min(MIN_POWER, power));
            motorToDesiredPower.put(m, Math.min(MIN_POWER, power));
        }

        boolean active = true;

        long startTime = System.currentTimeMillis();
        long totalMotorSetTime = 0;
        int loopCount = 0;

        double prevYawError = 0;
        double minYawAdjustment = 0;
        double maxYawAdjustment = 0;

        double yawErrorIntegral = 0;

        double prevHoldHeadingPowerAdjustment = 0;
        int oscillations = 0;


        while (System.currentTimeMillis() < stopTime && active && opModeIsActive())
        {
            loopCount++;

            StringBuilder message = new StringBuilder();
            message.append("Moving " + direction + "\n");

            active = true;
            double maxTicksRemaining = 0;

            for(DcMotor m : motorToEncoder.keySet())
            {
                int encoderTicks = motorToEncoder.get(m);
                int currentPosition = m.getCurrentPosition();
                motorToPosition.put(m, currentPosition);

                // Sometimes the motor is not busy right at the start, so assume we are
                // busy if we haven't moved half of the encoder ticks.
                // Stop when we get close.
                double ticksRemaining = Math.abs(encoderTicks - currentPosition);
                maxTicksRemaining = Math.max(maxTicksRemaining, ticksRemaining);

                // This is about 1/8 of an inch.
                if (ticksRemaining <= 10)
                {
                    active = false;
                    break;
                }

                // If we are rotating, then we can stop the loop once we achieve the desired yaw
                // change.
                if (targetYawChange != null) {
                    double yawTrueDiff = robot.imu.getTrueDiff(initialYaw);

                    message.append("initYaw=" + hundredths(initialYaw) + "  ");
                    message.append("true diff=" + hundredths(yawTrueDiff) + "\n");

                    double currentYawChange = yawTrueDiff;
                    if (Math.abs(currentYawChange) >= Math.abs(targetYawChange)) {
                        active = false;
                    } else {
                        active = true;
                    }

                    /**
                     *  When targeting values close to 180 or -180, we have to worry about the
                     *  difference becoming negative or positive. So if the yaw change is large (
                     *  >150) and the signs are different, then we've wrapped around.
                     **/

                    if ((Math.abs(currentYawChange) > 150) && Math.signum(currentYawChange)
                            != Math.signum(targetYawChange)) {
                        active = false;
                    }

                    if (active)
                    {
                        /**
                         * If we have moved half of our target distance, then extrapolate what
                         * the encoders will be when we get to the target rotation and set
                         * the encoders to that. This also lets us ramp down the power.
                         **/

                        if (Math.abs(currentYawChange) >=  Math.abs(targetYawChange) / 2) {

                            // Adjusts the encoder ticks based on what we have done so far.
                            double fractionComplete = Math.abs(currentYawChange) /
                                    Math.abs(targetYawChange);
                            int estimatedTargetedTicks = (int)Math.round(currentPosition /
                                    fractionComplete);
                            m.setTargetPosition(estimatedTargetedTicks);
                            motorToEncoder.put(m, estimatedTargetedTicks);
                        }
                    }
                }
                encoderTicks = motorToEncoder.get(m);

                // By default stay at the current power so that we don't set one motor to 0 and brake.
                double powerToSet = motorToDesiredPower.get(m);
                if (((encoderTicks < 0) && (currentPosition > encoderTicks)) ||
                        ((encoderTicks > 0) && (currentPosition < encoderTicks))) {
                    // Accelerate slowly at the start.
                    long millisSinceStart = System.currentTimeMillis()-startTime;
                    double rampUpMaxPower = MIN_POWER + (millisSinceStart / RAMP_UP_MS);

                    /**
                     * Figuring out how to ramp down is more complicated.
                     * At a ramp of 500ms, we got to max power in 12 inches.
                     * distance = .5 * a * t^2
                     * Start ramping down from full power at 12 inches.
                     * 12 inches = .5 * a * .5^2
                     * a = 12 inches / .5^3.
                     * a = 96 inches per second per second.
                     * BUT ramping down isn't symmetric to ramping up because we have
                     * momentum, so we should increase RAMP_DOWN_MS even more.
                     **/

                    double inchesToEnd  = ticksRemaining * (1.0 / moveInchtoEnc(1));
                    double rampDownAcclerationInchesPerSecond2 = 96;
                    double secondsToEnd  = Math.sqrt(2.0 * inchesToEnd /
                            rampDownAcclerationInchesPerSecond2);
                    double msToEnd = 1000 * secondsToEnd;
                    double rampDownMaxPower = MIN_POWER + (msToEnd) / RAMP_DOWN_MS;

                    powerToSet = Math.min(power,
                            Math.min(rampUpMaxPower, rampDownMaxPower));

                    message.append("msSinceStart: " + Math.round(millisSinceStart) + "  ");
                    message.append("msToEnd: " + Math.round(msToEnd) + "\n");
                    message.append("inchesToEnd: " + hundredths(inchesToEnd) + "\n");
                    message.append("rampUpPwr: " + hundredths(rampUpMaxPower) + "  ");
                    message.append("rampDownPwr: " + hundredths(rampDownMaxPower) + "\n");
                }

                message.append("Power=" + hundredths(powerToSet) + ", curTicks=" +
                        currentPosition + " targetTicks=" + encoderTicks + "\n");

                motorToDesiredPower.put(m, powerToSet * motorToPowerRatio.get(m));
            }

            // How to hold a heading.
            if (holdHeading) {
                double powerFL = motorToDesiredPower.get(robot.driveTrain.motorFL);
                double powerBL = motorToDesiredPower.get(robot.driveTrain.motorBL);
                double powerFR = motorToDesiredPower.get(robot.driveTrain.motorFR);
                double powerBR = motorToDesiredPower.get(robot.driveTrain.motorBR);
                double currentMaxPower = absMax(powerFL, powerBL, powerFR, powerBR);

                // This is the P in PID
                double yawError = robot.imu.getTrueDiff(headingToHold);

                // Keep track of oscillations for debugging.
                if (Math.signum(yawError) != Math.signum(prevYawError)) {
                    oscillations++;
                }

                // This is the I in PID.
                yawErrorIntegral = yawErrorIntegral + yawError;

                // This is the D in PID.
                double yawErrorDerivative = yawError - prevYawError;

                // Standard PID : output = (Kp * error) + (Ki * integral) + (Kd * derivative);
                double holdHeadingPowerAdjustment = PID_KP * yawError +
                        PID_KI * yawErrorIntegral + PID_KD * yawErrorDerivative;

                // Remember the raw adjustment before we clamp it down.
                double rawHoldHeadingPowerAdjustment = holdHeadingPowerAdjustment;

                // Scale the adjustment by the current power, not the maximum.
                // This is important when we are ramping up and down.
                holdHeadingPowerAdjustment = holdHeadingPowerAdjustment * currentMaxPower;

                // Cap the adjustment relative to the maximum limits and what we had set last time.
                holdHeadingPowerAdjustment = Math.min(maxHoldHeadingPowerAdjustment, holdHeadingPowerAdjustment);
                holdHeadingPowerAdjustment = Math.max(-maxHoldHeadingPowerAdjustment, holdHeadingPowerAdjustment);
                holdHeadingPowerAdjustment = Math.min(holdHeadingPowerAdjustment,
                        prevHoldHeadingPowerAdjustment + maxAdjustmentPerIteration);
                holdHeadingPowerAdjustment = Math.max(holdHeadingPowerAdjustment,
                        prevHoldHeadingPowerAdjustment - maxAdjustmentPerIteration);

                // Keep track of the min and max for telemetry at the end.
                maxYawAdjustment = Math.max(holdHeadingPowerAdjustment, maxYawAdjustment);
                minYawAdjustment = Math.min(holdHeadingPowerAdjustment, minYawAdjustment);

                message.append("yawError: " + hundredths(yawError) + "\n");
                message.append("pidAdjustment: " + hundredths(holdHeadingPowerAdjustment) + "\n");

                // To rotate clockwise (+ yaw)
                //   encFL & encBL are positive.
                //   encFR & encBR are negative.

                // To rotate counterclockwise (- yaw)
                //   encFL & encBL are negative.
                //   encFR & encBR are positive

                // So ...
                // If yawError is positive, then we need to rotate counter clockwise
                // If yawError is negative, then we need to rotate clockwise

                // A positive adjustment means that we rotate clockwise and adjust powers as
                // Add to encFL & encBL
                // Subtract from encFR & encBR

                // When we are speeding up or slowing down, don't let the adjustment get too big.
                double maxPower = Math.min(1,
                        (1 + (maxHoldHeadingPowerAdjustment/ power)) * currentMaxPower);

                powerFL = adjustPower(powerFL, holdHeadingPowerAdjustment, maxPower);
                powerBL = adjustPower(powerBL, holdHeadingPowerAdjustment, maxPower);
                powerFR = adjustPower(powerFR, -holdHeadingPowerAdjustment, maxPower);
                powerBR = adjustPower(powerBR, -holdHeadingPowerAdjustment, maxPower);
                motorToDesiredPower.put(robot.driveTrain.motorFL, powerFL);
                motorToDesiredPower.put(robot.driveTrain.motorBL, powerBL);
                motorToDesiredPower.put(robot.driveTrain.motorFR, powerFR);
                motorToDesiredPower.put(robot.driveTrain.motorBR, powerBR);

                // Log this so we can analyze it later.
                appendPid(startTime, csvBuffer,
                        yawError, prevYawError, rawHoldHeadingPowerAdjustment,
                        holdHeadingPowerAdjustment, prevHoldHeadingPowerAdjustment,
                        yawErrorIntegral, motorToDesiredPower, motorToPosition);


                prevHoldHeadingPowerAdjustment = holdHeadingPowerAdjustment;
                prevYawError = yawError;
            }

            

            // Set the motor powers all at the end, so we are adjusting at the same time.
            long startMotorSet = System.currentTimeMillis();
            for (DcMotor m: motorToDesiredPower.keySet()) {
                m.setPower(motorToDesiredPower.get(m));
            }
            totalMotorSetTime += System.currentTimeMillis() - startMotorSet;

            robot.log(message.toString());

            // If all of the encoders are close, then set a timeout to 1 second from now.
            if(maxTicksRemaining < 100){
                stopTime = Math.min(stopTime,System.currentTimeMillis() + 1000);
            }

            // If the motors are very close, then timeout 20 ms from now.
            if(maxTicksRemaining < 10){
                stopTime = Math.min(stopTime,System.currentTimeMillis() + 20);
            }
        }

        for (DcMotor m : motorToEncoder.keySet())
        {
            m.setPower(0);
            telemetry.addData(m.getDeviceName(), m.getCurrentPosition());
        }

        // Log all of the PID details to a CSV file.
        csvBuffer.append("\n");
        csvBuffer.append("\n");
        csvBuffer.append("Loop count," + loopCount + "\n");
        csvBuffer.append("Yaw," + robot.imu.getYaw() + "\n");
        csvBuffer.append("Initial Yaw," + initialYaw + "\n");
        csvBuffer.append("Yaw change," + robot.imu.getTrueDiff(initialYaw) + "\n");
        csvBuffer.append("Max yaw adjustment," + maxYawAdjustment + "\n");
        csvBuffer.append("Min yaw adjustment," + minYawAdjustment + "\n");
        csvBuffer.append("Oscillations," + oscillations + "\n");
        File file = AppUtil.getInstance().getSettingsFile("pid-" + direction + "-" +
                System.currentTimeMillis() + ".csv");
        try(FileOutputStream out = new FileOutputStream((file))){
            out.write(csvBuffer.toString().getBytes());
        } catch(IOException e) {
            telemetry.addLine(e.toString());
        }

        telemetry.addData("Loop count", loopCount);
        telemetry.addData("totalMotorSetTime", totalMotorSetTime);
        telemetry.addData("Yaw: ", robot.imu.getYaw());
        telemetry.addData("Initial Yaw: ", initialYaw);
        telemetry.addData("Yaw change: ", robot.imu.getTrueDiff(initialYaw));
        if (maxYawAdjustment != 0) {
            telemetry.addData("Max yaw adjustment", maxYawAdjustment);
        }
        if (minYawAdjustment != 0) {
            telemetry.addData("Min yaw adjustment", minYawAdjustment);
        }
        if (oscillations > 0) {
            telemetry.addData("Yaw oscillations", oscillations);
        }
        telemetry.addData("Method time: ", (System.currentTimeMillis()
                - methodStartMillis) + " ms");
        telemetry.addData("PID file", file.getAbsolutePath());
        telemetry.update();
    }

    private final DecimalFormat D1 = new DecimalFormat("0.0");
    private final DecimalFormat D2 = new DecimalFormat("#.00");
    private final DecimalFormat D3 = new DecimalFormat("#.000");
    private final DecimalFormat D4 = new DecimalFormat("#.0000");

    // Store the previous encders so we can see their velocities.
    Map<DcMotor, Integer> prevMotorToPosition;

    // Write out one row to the CSV file.
    private void appendPid(long millis,
                           StringBuilder csvBuffer,
                           double yawError,
                           double prevYawError,
                           double rawAdjust,
                           double adjust,
                           double prevAdjust,
                           double yawErrorIntegral,
                           Map<DcMotor, Double> motorToDesiredPower,
                           Map<DcMotor, Integer> motorToPosition
                           ) {

        DcMotor motorFL = robot.driveTrain.motorFL;
        DcMotor motorBL = robot.driveTrain.motorBL;
        DcMotor motorFR = robot.driveTrain.motorFR;
        DcMotor motorBR = robot.driveTrain.motorBR;

        double powerFL = motorToDesiredPower.get(motorFL);
        double powerBL = motorToDesiredPower.get(motorBL);
        double powerFR = motorToDesiredPower.get(motorFR);
        double powerBR = motorToDesiredPower.get(motorBR);

        double durationSec = (System.currentTimeMillis() - millis)/ 1000.0;

        double errorDelta = Math.abs(yawError) - Math.abs(prevYawError);
        double adjDelta = adjust - prevAdjust;

        csvBuffer.append("\n" +
                D3.format(durationSec) +
                "," + D1.format(yawError) +
                "," + D1.format(prevYawError) +
                "," + D1.format(errorDelta) +
                "," + D3.format(adjust) +
                "," + D3.format(rawAdjust) +
                "," + D3.format(prevAdjust) +
                "," + D3.format(adjDelta) +
                "," + D1.format(yawErrorIntegral));

        if (prevMotorToPosition == null) {
            prevMotorToPosition = new LinkedHashMap<>(motorToPosition);
            prevMotorToPosition.put(motorFL, 0);
            prevMotorToPosition.put(motorBL, 0);
            prevMotorToPosition.put(motorFR, 0);
            prevMotorToPosition.put(motorBR, 0);
        }

        csvBuffer.append("," + D3.format(Math.abs(powerFL)));
        csvBuffer.append("," + D3.format(Math.abs(powerBL)));
        csvBuffer.append("," + D3.format(Math.abs(powerFR)));
        csvBuffer.append("," + D3.format(Math.abs(powerBR)));
        csvBuffer.append("," + Math.abs(motorToPosition.get(motorFL) - prevMotorToPosition.get(motorFL)));
        csvBuffer.append("," + Math.abs(motorToPosition.get(motorBL) - prevMotorToPosition.get(motorBL)));
        csvBuffer.append("," + Math.abs(motorToPosition.get(motorFR) - prevMotorToPosition.get(motorFR)));
        csvBuffer.append("," + Math.abs(motorToPosition.get(motorBR) - prevMotorToPosition.get(motorBR)));

        prevMotorToPosition = new LinkedHashMap<>(motorToPosition);
    }


    private double absMax(double... values) {
        double max = 0;
        for (double value: values) {
            max = Math.max(Math.abs(value), max);
        }
        return max;
    }

    private double adjustPower(double power, double adjustment, double maxAbsolutePower) {
        double adjustedPower = power + adjustment;
        if (adjustedPower > maxAbsolutePower) {
            return maxAbsolutePower;
        } else if (adjustedPower < -maxAbsolutePower) {
            return -maxAbsolutePower;
        } else {
            return adjustedPower;
        }
    }

    final DecimalFormat hundredthsFormat = new DecimalFormat("0.00");
    private String hundredths(double value) {
        return hundredthsFormat.format(value);
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
    // A positive value is clockwise.
    // A negative value is counterclockwise.
    public void rotateToAbsoluteYaw(double yawDegrees)
    {
        double targetYawChange = robot.imu.getTrueDiff(yawDegrees);
        rotate(targetYawChange);
        setHeadingToHold(yawDegrees);
    }


    // positive is left, negative is right
    // This is private because we need make sure headingToHold gets updated.
    private void rotate(double degrees)
    {
        rotate(degrees, 0.5);

    }

    // This is private because we need make sure headingToHold gets updated.
    private void rotate(double degrees, double power)
    {
        // Flip the direction of the encoders if degrees is negative.
        int encoderSign = 1;
        if (degrees < 0) {
            encoderSign = -1;
        }

        int maxEncoder = 5000;
        moveWithEncoders("ROTATE", power, 10000,
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
        final double DIST_BTWN_WHEELS = 15.5;
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

        // Remember which direction we are heading.
        setHeadingToHold(endingAbsoluteYaw);
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


    public void moveClampOutInAuto() {
        robot.output.moveClampOutOfRobot();

        robot.output.moveElbowToPosition(robot.output.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);
    }


    public void grabStoneInAuto(int backwards)
    {
        moveForwardByInches(.5, 12);

        robot.output.closeClampFully();

        robot.output.moveElbowToPosition(robot.output.ELBOW_POSITION_OUTSIDE_ROBOT_AND_PARTIALLY_UP);

        moveBackwardByInches(0.6, backwards);
    }


    public void driveUnderBridge(boolean isBlue, int inches)
    {
        if (isBlue)
            strafeLeftByInches(1, inches);
        else
            strafeRightByInches(1, inches);
    }

    public void strafeToStone(boolean isBlue)
    {
        if (isBlue)
            strafeRightByInches(0.7, 8);
        else
            strafeLeftByInches(0.7, 8);
    }


    public void dropStone()
    {
        // TODO: Implement me.
        // turn wrist down
        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_OUTSIDE_ROBOT_DOWN_A_LITTLE);
        robot.output.openClampFully();
        robot.output.stopClamp();
    }


    public void pullFoundationAndPark(boolean isBlue, boolean side)
    {
        robot.foundationHooks.lowerHooks();
        moveBackwardByInches(0.8, 42);

        if (isBlue) {
            rotate(20);
            robot.foundationHooks.raiseHooks();
            strafeRightByInches(1, 10);
            rotateToAbsoluteYaw(-70);

            if (side) {
                moveBackwardByInches(1, 28);
                strafeRightByInches(1, 10);
                moveBackwardByInches(1, 8);
            }
            else
                moveBackwardByInches(1, 36);
        }

        else {
            rotate(-20);
            robot.foundationHooks.raiseHooks();
            strafeLeftByInches(1, 10);
            rotateToAbsoluteYaw(70);

            if (side) {
                moveBackwardByInches(1, 28);
                strafeLeftByInches(1, 10);
                moveBackwardByInches(1, 8);
            }
            else
                moveBackwardByInches(1, 36);
        }
    }


    public void grabAndTurn(boolean isBlue) {
        if (isBlue) {
            grabStoneInAuto(7);
            rotateToAbsoluteYaw(-70);
        }
        else {
            grabStoneInAuto(7);
            rotateToAbsoluteYaw(70);
        }
    }


    public boolean isSkystone(boolean isBlue)
    {
        long stopTime = System.currentTimeMillis() + 800; // TODO: Could shorten this to save time
        while (System.currentTimeMillis() < stopTime)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                int i = 1;

                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.addData("  Angle", String.format(" %.0f degrees",
                            recognition.estimateAngleToObject(AngleUnit.DEGREES)));
                    telemetry.update();

                    if (recognition.getLabel().equals("Skystone") && recognition.getConfidence() > .20) {
                        double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (isBlue) {
                            if ((angle > -25) && (angle < 15))
                                return true;
                        } else {
                            // Red side
                            if ((angle > -10) && (angle < 25))
                                return true;
                        } // return false if skystone seen but not in angle, saves time
                    }
                    i++;
                }
            }
        }
        return false;
    }

    public void printRecognitions()
    {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
            Collections.sort(updatedRecognitions, new Comparator<Recognition>() {
                        @Override
                        public int compare(Recognition o1, Recognition o2) {
                            return Double.compare(o1.estimateAngleToObject(AngleUnit.DEGREES),
                                    o2.estimateAngleToObject(AngleUnit.DEGREES));
                        }
                    });
                    // step through the list of recognitions and display boundary info.
            int i = 1;

            StringBuilder sb = new StringBuilder();

            for (Recognition recognition : updatedRecognitions) {
                sb.append(recognition.getLabel() + " " + hundredths(recognition.estimateAngleToObject(AngleUnit.DEGREES))
                        + " " + hundredths(recognition.getConfidence()) + "\n");
            }

            telemetry.addData(sb.toString(), " ");
            telemetry.update();

        }

    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        log("Initializing Vuforia");
        long startMillis = System.currentTimeMillis();

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Allows us to capture pictures from the phone.
        vuforia.enableConvertFrameToBitmap();
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(4);
        vuforia.enableConvertFrameToBitmap();

        log("Initializing Vuforia took " + (System.currentTimeMillis() - startMillis) + " ms");
    }


    public Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        List<Integer> formats = new ArrayList<>();
        int format = 0;
        for (int i = 0; i < numImages; i++) {
            format = picture.getImage(i).getFormat();
            formats.add(format);
            rgb = picture.getImage(i);
            if (format == PIXEL_FORMAT.RGB565) {
                break;
            }
        }

        if (format != PIXEL_FORMAT.RGB565) {
            telemetry.addLine("Didn't find correct RGB format (" + PIXEL_FORMAT.RGB565 +
                    "). Got formats=" + formats);
            telemetry.update();
        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        picture.close();
        return imageBitmap;
    }

    /**
     * Calculates the average Blue divided by Red + Green for the region of the
     * image. Yellow stones will have a smaller value relative to Skystones. We can use this to
     * compare the middle of three different stones to find the one that is most likely the
     * Skystone.
     */
    public double calculateBlueRatioForRegion(Bitmap bitmap, int centerX, int centerY,
                                              int width, int height) {

        int leftPixel = centerX - (width/2);
        int topPixel = centerY - (height/2);

        long startMillis = System.currentTimeMillis();
        // Size is 1280 x 720. Starting from the top left.
        double totalR = 0;
        double totalG = 0;
        double totalB = 0;

        for (int i = leftPixel; i < (leftPixel + width); i++) {
            for (int j = topPixel; j < (topPixel + height); j++) {
                int pixel = bitmap.getPixel(i, j);
                int redPixel = red(pixel);
                int greenPixel = green(pixel);
                int bluePixel = blue(pixel);

                totalR += redPixel;
                totalG += greenPixel;
                totalB += bluePixel;
            }
        }
        long durationMillis = System.currentTimeMillis() - startMillis;

        int total = width * height;

        double blueRatio = totalB / (Math.max(1, totalR + totalG));

//        telemetry.addLine("Calculated blue ratio for region in " + durationMillis + " ms");
//        telemetry.addData("leftPixel=", leftPixel);
//        telemetry.addData("topPixel=", topPixel);
//        telemetry.addData("width=", width);
//        telemetry.addData("height=", height);
//        telemetry.addData("avg R=", Math.round(totalR / total));
//        telemetry.addData("avg G=", Math.round(totalG / total));
//        telemetry.addData("avg B=", Math.round(totalB / total));
//        telemetry.addData("B/(R+G)", blueRatio);
//        telemetry.update();

        return blueRatio;
    }

    public void colorRegion (Bitmap bitmap, int centerX, int centerY,
                             int width, int height, int color){
        int leftPixel = centerX - (width/2);
        int topPixel = centerY - (height/2);

        for (int i = leftPixel; i < (leftPixel + width); i++) {
            for (int j = topPixel; j < (topPixel + height); j++) {
                bitmap.setPixel(i, j, color);
            }
        }
    }

    public StonePosition getSkystonePosition() throws InterruptedException {
        double max = 0;
        StonePosition skystone = null;
        Bitmap bm = getBitmap();
        for(StonePosition pos : StonePosition.values()){
            double ratio = calculateBlueRatioForRegion(bm, pos.getCenterX(), pos.getCenterY(), WIDTH_OF_RECTANGLE, HEIGHT_OF_RECTANGLE);
            if(ratio >= max){
                max = ratio;
                skystone = pos;
            }

            telemetry.addData("Stone " + pos + " blue ratio", ratio);

            // This shows us what regions we are looking at in the images.
            colorRegion(bm, pos.getCenterX(), pos.getCenterY(), WIDTH_OF_RECTANGLE, HEIGHT_OF_RECTANGLE, pos.getColor());
        }

        telemetry.addData("SkyStone", skystone);
        telemetry.update();

        // This shows us what region is the SkyStone.
        colorRegion(bm, skystone.getCenterX(), skystone.getCenterY(), WIDTH_OF_RECTANGLE, HEIGHT_OF_RECTANGLE, Color.WHITE);

        // Enable this code to write out the images that we are detecting and what
        // portion of the image that we are checking.
//        File file = AppUtil.getInstance().getSettingsFile("bmPic-" + System.currentTimeMillis() + ".png");
//        try(FileOutputStream out = new FileOutputStream((file))){
//            bm.compress(Bitmap.CompressFormat.PNG, 100, out);
//        } catch(IOException e) {
//            telemetry.addLine(e.toString());
//        }
//        telemetry.addData("file", file.getAbsolutePath());
//        telemetry.update();

        return skystone;
    }


    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         */
        tfod.activate();
    }
}
