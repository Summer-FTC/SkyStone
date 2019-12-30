/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "TensorFlow", group = "6209")
public class TensorFlowSkyStone extends BaseLinearOpMode{
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private int skystonePosition = 3;

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
         //   " -- YOUR NEW VUFORIA KEY GOES HERE  --- ";

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

    @Override
    public void runOpMode() throws InterruptedException
    {
        super.initialize();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector())
        {
            initTfod();
        }

        else
        {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // TODO: This assumes blue currently.
        boolean isBlue = true;
        if (opModeIsActive())
        {
            moveForwardByInches(1, 19);
            if (isSkystone(isBlue))
            {
                grabStoneInAuto();
                //driveUnderBridge();
               // dropStone();
                // pull foundation
                // park
            }
            else
            {
                strafeRightByInches(1, 8);
                if(isSkystone(isBlue))
                {
                    grabStoneInAuto();
                }
                else
                {
                    strafeRightByInches(1, 8);
                    grabStoneInAuto();
                }
            }

        }

    }

    public boolean isSkystone(boolean isBlue)
    {
        long stopTime = System.currentTimeMillis() + 1000; // TODO: Maybe this needs to be longer.
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

                    if (recognition.getLabel().equals("Skystone") && recognition.getConfidence() > .50) {
                        double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        if (isBlue) {
                            if ((angle > -15) && (angle < 25)) {
                                return true;
                            }
                        } else {
                            // Red sidie
                            if ((angle > -25) && (angle < 15)) {
                                return true;
                            }
                        }
                    }
                }
            }
        }
        return false;
    }

    public void runOpModeOld() throws InterruptedException
    {

        super.initialize();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // add move to location first
        // or make this a method

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.

                    // move up to the stones
                    moveForwardByInches(0.5, 16);

                    // STRAFE HERE? DISTANCE ????
                    telemetry.addData("", "Sleeping");
                    telemetry.update();
                    sleep(5000);

                    telemetry.addData("", "Done sleeping");
                    telemetry.update();


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
                            telemetry.update();

                            if (recognition.getLabel().equals("Skystone") && recognition.getConfidence() > .50)
                            {
                                telemetry.addData("", "Step 1");
                                telemetry.update();
                                grabStoneInAuto();
                             //   driveUnderBridge();
                             //   dropStone();
                            }

                            else {
                                telemetry.addData("", "Step 2");
                                telemetry.update();
                                // Todo: Make this customizable Left/Right corresponding with Blue/Red
                                // Todo: make the distance a constant and figure out how far it is to get to the next block


                                strafeRightByInches(0.7, 8);

                                if (recognition.getLabel().equals("Skystone") && recognition.getConfidence() > .50)
                                {
                                    grabStoneInAuto();
                                 //   driveUnderBridge();
                                 //   dropStone();
                                }

                                // Should we make it check the 3rd stone or just assume if it didn't detect one
                                // of the first two then its the 3rd?
                                else
                                {
                                    telemetry.addData("", "Step 3");
                                    telemetry.update();
                                    // Todo: make the inches same constant as above
                                    strafeRightByInches(0.7, 8);
                                    grabStoneInAuto();
                                 //   driveUnderBridge();
                                 //   dropStone();

                                }

                            }
                        }
                    }
                }

                // Todo: add foundation run auto here?
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    public void driveUnderBridge()
    {
        // Todo: figure out these distances
        moveBackwardByInches(0.5, 5);
        strafeLeftByInches(0.75, 60);
    }

    // Todo: change positions 1 2 3 to constants
    // Todo: figure out if we need this method if we're strafing?
    public void moveToStone(int position) throws InterruptedException
    {
        if (position == 1)
        {
            // get stone in position 1;
            strafeRightByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
            grabStoneInAuto();
        }

        else if (position == 2)
        {
            // get stone in position 2;
            strafeLeftByInches(0.5, 3);
            moveForwardByInches(0.5, 5);
            grabStoneInAuto();
        }
        else
        {
            // get stone in position 3;
            strafeLeftByInches(0.5, 10);
            moveForwardByInches(0.5, 5);
            grabStoneInAuto();
        }
    }

    // Todo: might not need this method
    public int getSkystonePosition()
    {
        return skystonePosition;
    }

    /**
     * Initialize the Vuforia localization engine.
     */

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}