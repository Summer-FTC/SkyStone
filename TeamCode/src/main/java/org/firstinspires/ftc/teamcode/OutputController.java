package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OutputController
{

    public DcMotor motorLift;
    public Servo elbow1;
    public Servo elbow2;
    public Servo wrist;
    public CRServo clamp;
    int position = 1;

    HardwareMap hwMap;
    Telemetry telemetry;


    public void init(HardwareMap hwMap, Telemetry telemetry)
    {
        this.hwMap = hwMap;
        this.telemetry = telemetry;

        motorLift = hwMap.dcMotor.get("motorLift");

        // elbows flip
        // 1 and 2 should work simultaneously
        elbow1 = hwMap.servo.get("elbow1");
        elbow2 = hwMap.servo.get("elbow2");
        // wrist rotates block
        wrist = hwMap.servo.get("wrist");
        // clamp opens and closes on block
        clamp = hwMap.crservo.get("clamp");

        telemetry.addData("Output Servo Initialization Complete", "");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Output Motor Initialization Complete", "");
    }

    public void setLiftPower(double liftPower) {
            motorLift.setPower(liftPower);
    }

    public void stopOutputMotors()
    {
        motorLift.setPower(0);
    }


    public void moveToPosition(int newPos)
    {
        if (newPos == 1)
        {
            // inside

            telemetry.addData("" + position, "");

            if (position == 3)
            {

                threeToTwo();
            }

            twoToOne();

            position = 1;

            telemetry.addData("" + position, "");
            telemetry.update();
        }

        else if (newPos == 2)
        {
            // outside high

            telemetry.addData("" + position, "");

            // go outward from position 1
            if (position == 1)
                oneToTwo();

            // raise from position 3
            if (position == 3)
                threeToTwo();

            position = 2;

            telemetry.addData("" + position, "");
            telemetry.update();

        }

        else if (newPos == 3)
        {
            // outside low
            // lower from position 2;

            telemetry.addData("" + position, "");

            if (position == 1) {
                oneToTwo();

                setLiftPower(-0.5);
                sleep(1000);
                setLiftPower(0);

                twoToThree();
            }

            if (position == 2)
                twoToThree();

            position = 3;

            telemetry.addData("" + position, "");
            telemetry.update();
        }
    }

    // since intake not working right now, this will only be used in auto to go out
    public void oneToTwo() {
        closeClamp();
        sleep(1000);

        setLiftPower(0.5);
        sleep(1000);
        setLiftPower(0);

        setElbowPositions(0.3);
        wrist.setPosition(0.5); // 90 degrees
        setElbowPositions(0.7);
        wrist.setPosition(1);
    }

    // won't use in AML2
    public void twoToOne() {
        setLiftPower(0.5);
        sleep(1000);
        setLiftPower(0);

        wrist.setPosition(0.5);
        setElbowPositions(0.3);
        wrist.setPosition(0);

        setLiftPower(-0.5);
        sleep(1000);
        setLiftPower(0);

        setElbowPositions(0);

        openClamp();
        sleep(500);
    }

    public void threeToTwo() {
        closeClamp();
        sleep(1000);

        setElbowPositions(0.7);
    }

    public void twoToThree()
    {
        setElbowPositions(1);

        openClamp();
        sleep(1000);
    }

    public void openClamp()
    {
        clamp.setPower(1);
    }

    public void closeClamp()
    {
        clamp.setPower(-1);
    }

    public void stopClamp()
    {
        clamp.setPower(0);
    }

    public void setElbowPositions(double pos)
    {
        elbow1.setPosition(pos);
        elbow2.setPosition(pos);
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}