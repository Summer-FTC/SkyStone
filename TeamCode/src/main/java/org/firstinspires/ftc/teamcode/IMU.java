/* Copyright (c) 2017 FIRST. All rights reserved.
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

// ^^ lol idk if we need this but they had it last year

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
/**
 * {@link IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

@Autonomous (name = "IMU", group = "6209")
@Disabled
public class IMU extends LinearOpMode
{
    BNO055IMU IMU;
    Orientation angles;
    Acceleration gravity;

    public IMU (BNO055IMU imu)
    {
        IMU = imu;
    }

    // Just basic init stuff; run in robot init method
    public void IMUinit(HardwareMap map)
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        IMU = map.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);
    }

    //returns yaw between -179.9999 and 180 degrees
    public double getYaw()
    {
        // yaw is side to side (turning)
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return -Double.parseDouble(formatAngle(angles.angleUnit, angles.firstAngle));
    }

    //returns yaw between -179.9999 and 180 degrees
    public double getPitch()
    {
        // pitch is up and down like the nose of an airplane
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.thirdAngle));
    }

    public double getRoll()
    {
        // roll is side to side rotation (think about an airplane doing a barrel roll)
        angles = IMU.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return Double.parseDouble(formatAngle(angles.angleUnit, angles.secondAngle));
    }

    // Calculates the smallest difference between a specified angle and the current angle
    public double getTrueDiff(double origAngle)
    {
        double currAngle = getYaw();
        if (((currAngle >= 0) && (origAngle >= 0) )|| ((currAngle <= 0) && (origAngle <= 0)))
            return currAngle - origAngle;
        else if (Math.abs(currAngle - origAngle) <= 180)
            return currAngle - origAngle;
        else if (currAngle > origAngle)
            return -(360 - (currAngle - origAngle));
        else
            return (360 + (currAngle - origAngle));

    }

    String formatAngle(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit, double angle)
    {
        return formatDegrees(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees)
    {
        return String.format(Locale.getDefault(), "%.1f", org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES.normalize(degrees));
    }

    public void runOpMode()
    {

    }



}
