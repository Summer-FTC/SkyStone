package org.firstinspires.ftc.teamcode;

public class OpMode
{
    boolean operationOnly = false;
@TeleOp (name= "VenomTeleOp", group= "6209")
    public static class VenomTeleOp extends OpMode{
    @Override
    public void init(){
        robot = new VenomHardwareMap;
        telemetry.addData("Robot","Initializing...");
        telemetry.update();
        robot.init(hardwareMap,telemetry);
        super.init();
    }
    public void start() {// do anything we need in start button
    }
    public void stop(){
        robot.stop();
    }

    VenomHardwareMap hwMap = null;
    Telemetry telemetry = new telemetry;

    ElapsedTime timer = new timer;
    @Override
    public void loop(telemetry.addData("Robot","Running TeleOp")){
        doDrive();
    }

    void doDrive(){
    double leftX = (gamepad1.left_stick_x);
    double leftY = (-gamepad1.left_stick_y);
    double rightX = (-gampead.right_stick_x);
    double rightY = (-gamepad1.right_stick_y);

    double forward, rotate, sideways;

    telemetry.addData("Drive: ", "F=" + forward + "S=" + sideways + "R=" + rotate);
    telemetry.update();
    }

    
}

}



