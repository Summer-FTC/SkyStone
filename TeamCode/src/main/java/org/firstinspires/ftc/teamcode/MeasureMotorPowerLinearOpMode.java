package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * This class runs each motor at various speeds and reports the encoders
 * per second at that speed. These calculations can help to adjust the
 * motor powers in TeleOp so that the motors are moving at the expected
 * velocity.
 */
@Autonomous(name = "MeasureMotorPowerLinearOpMode", group = "6209")
@Disabled
public class MeasureMotorPowerLinearOpMode extends BaseLinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        telemetry.addData("Before moving", "");

        Map<String,DcMotor> motorNameToMotor = new LinkedHashMap<>();

        motorNameToMotor.put("motorFL", robot.driveTrain.motorFL);
        motorNameToMotor.put("motorFR", robot.driveTrain.motorFR);
        motorNameToMotor.put("motorBL", robot.driveTrain.motorBL);
        motorNameToMotor.put("motorBR", robot.driveTrain.motorBR);

        double[] powers = new double[]{0.15, 0.25, 0.375, 0.5, 0.65, 0.80, 1.0};
        StringBuilder results = new StringBuilder();
        results.append("motor/power, ");
        for (double power: powers)
        {
            results.append("," + power);
        }
        results.append("\n");

        Map<String,double[]> motorClicksAtPower = new LinkedHashMap<>();
        for (String motorName : motorNameToMotor.keySet()) {

            results.append(motorName);
            DcMotor motor = motorNameToMotor.get(motorName);
            double[] clicksPerSecondAtPower = new double[powers.length];
            motorClicksAtPower.put(motorName, clicksPerSecondAtPower);
            for (int i = 0 ; i < powers.length; i++)
            {
                double power = powers[i];
                clicksPerSecondAtPower[i] = clicksPerSecondAtPower(motor, power);
                results.append(", " + clicksPerSecondAtPower[i]);
            }

            motor.setPower(0);
            results.append("\n");
        }
        log(results.toString());
        sleep(30 * 1000);

        // TODO: use motorClicksAtPower to calculate how much to adjust the
        // power based on each motor.
    }

    private double clicksPerSecondAtPower(DcMotor motor, double power)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(power);
        sleep(200);

        int before = motor.getCurrentPosition();

        int millis = 600;
        sleep(millis);
        int clicks = Math.abs(motor.getCurrentPosition() - before);

        double clicksPerSecond = (1000.0/millis) * clicks;
        return clicksPerSecond;
    }
}
