package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class BaseLinearOpMode extends LinearOpMode
{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AdlGeb7/////AAABmSRxygbWhU7JmYOsyl0bcLgAAIlM4tFb63K1a+swM0Z2qYggVDcwj/RDVakun/FOpm14tLjtU7UAmIRuCfA1Sah8YloIcX8O6+8Uj/BI3J9D/C3uiTBXzfLEA8Ml4c53WhR2GhQ1LJtfrZ1bjsmY5qksP7i0eaXfFUZ6s1elxJua5gVx4jbuVrh09yaGCfZ3GxymbY3S5ZJWDWiEB7RY5JIHGb01Ar30tzki47QL1YQKHkqM2u1Zm4aJl6/KedqOTc1EL3DNXAYb/jCj/Xnl2pzV7vAKUvhscgHA1MMHo5yPjL2mG6ySKKZnMr0tyjgwAYsyWA5syAi1Bgb+lqeUlsAaD3rssPZfPE0BzXV9dqzG";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    // This power will move the robot slowly.
    private static double MIN_POWER = 0.20;
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

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
            initTfod();

        else
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null)
            tfod.activate();

        log("BaseLinearOpMode::initialize complete");
    }


    public void moveWithEncoders(String direction, double power, int msTimeOut,
                                  int encFL, int encFR, int encBL, int encBR)
    {
        moveWithEncoders(direction, power, msTimeOut,
                encFL, encFR, encBL, encBR, null);
    }


    public void moveWithEncoders(String direction, double power, int msTimeOut,
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
            double maxTicksRemaining = 0;
            for(DcMotor m : motorToEncoder.keySet())
            {
                int encoderTicks = motorToEncoder.get(m);

                // Sometimes the motor is not busy right at the start, so assume we are
                // busy if we haven't moved half of the encoder ticks.
                // Stop when we get close.
                double ticksRemaining = Math.abs(encoderTicks - m.getCurrentPosition());
                maxTicksRemaining = Math.max(maxTicksRemaining, ticksRemaining);

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

            // If all of the encoders are close, then set a timeout to 1 second from now.

            if(maxTicksRemaining < 100){
                stopTime = Math.min(stopTime,System.currentTimeMillis() + 1000);
            }
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
        rotate(degrees, 0.3);

    }

    public void rotate(double degrees, double power)
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


    public void grabStoneInAuto(int backwards)
    {
        robot.output.moveClampOutOfRobot();

        moveForwardByInches(.5, 14);

        robot.output.moveElbowToPosition(robot.output.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);

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
            strafeRightByInches(1, 8);
        else
            strafeLeftByInches(1, 8);
    }


    public void dropStone()
    {
        // TODO: Implement me.
        // turn wrist down
        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_OUTSIDE_ROBOT_AND_DOWN);
        robot.output.startOpeningClamp();
        sleep(300); // can do this while lowering hooks
        // can have more time for OnlySkystone
        robot.output.stopClamp();
        robot.output.moveElbowToPosition(OutputController.ELBOW_POSITION_OUTSIDE_ROBOT_PARALLEL);
    }

    public void pullFoundationAndPark(boolean isBlue, boolean side)
    {
        robot.hooks.lowerHooks();
        moveBackwardByInches(0.8, 42);

        if (isBlue) {
            rotate(20);
            robot.hooks.raiseHooks();
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
            robot.hooks.raiseHooks();
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
            grabStoneInAuto(14);
            rotateToAbsoluteYaw(-70);
        }
        else {
            grabStoneInAuto(14);
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
                    telemetry.addData("  Angle", String.format(" %.0f degrees", recognition.estimateAngleToObject(AngleUnit.DEGREES)));
                    telemetry.update();

                    if (recognition.getLabel().equals("Skystone") && recognition.getConfidence() > .20) {
                        double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (isBlue) {
                            if ((angle > -15) && (angle < 25))
                                return true;
                        } else {
                            // Red side
                            if ((angle > -25) && (angle < 15))
                                return true;
                        }
                    }
                    i++;
                }
            }
        }
        return false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
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
    }
}
