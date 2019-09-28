package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class TrollBotAuto extends CustomLinearOpMode {

    ModernRoboticsI2cRangeSensor rangeSensor;
    boolean SkystoneDetected = false;

    public void isSkystone() {
        // if phone detects skystone
            return true;
        return false;
    }


    @Override
    public void runOpMode() {

        initizialize();
        waitForStart();

        try {

        } catch (Exception e) {
            stop();
        }

        // move to blocks
        moveToDistance(2);

        // Check first block
        turn(-90);
        if(isSkystone()) {
            driveBackward(2);
        }

        // Check second block
        if (!SkystoneDetected)
            turn(-90);
            if (isSkystone()) {
                driveBackward(2);

        // Check third block
        // But don't need to because has to be if first two aren't
        if (!SkystoneDetected)
            turn(-90);
            if (isSkystone()) {
                driveBackward(2);
            }
        }


        /*blockLocation = getBlockLocation();
        if (blockLocation.equals("LEFT")) {
            goForward(25.4558);
            turn(45.0);
            goForward(36);
            turn(90.0);
            goForward(24);
            depositMarker();
            turn(180.0);
            goForward(48.0);

            if (blockLocation.equals("LEFT")){
                turn(90.0);
                goForward(12.0);
                turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more
                // than 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be
                // facing the rover
            }

            else if (blockLocation.equals("RIGHT")){
                turn(90.0);
                goForward(36.0);
                turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees right
                // and go backwards around 24 sqrt 2 inches to be
                // around the center of the crater, could turn another
                // 90 degrees right to be facing the rover
            }

            else if (blockLocation.equals("CENTER")){
                turn(90.0);
                goForward(24.0);
                turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees
                // right and go backwards around 12 sqrt 2 inches
                // to be around the center of the crater, could
                // turn another 90 degrees right to be facing the rover
            }

        }

        if (blockLocation.equals("RIGHT")){
            goForward(25.4558);
            turn(-45.0);
            goForward(24.0);
            turn(90.0);
            goForward(36.0);
            turn(-90.0);
            depositMarker();
            turn(180.0);
            goForward(48.0);

            if (blockLocation.equals("LEFT")){
                turn(90.0);
                goForward(12.0);
                turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more than 45
                // degrees right or something and go like 12
                // sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

            else if (blockLocation.equals("RIGHT")){
                turn(90.0);
                goForward(36.0);
                turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                // You could turn a little less than 45 degrees right
                // and go backwards around 24 sqrt 2 inches to be around
                // the center of the crater, could turn another 90 degrees
                // right to be facing the rover
            }

            else if (blockLocation.equals("CENTER")){
                turn(90.0);
                goForward(24.0);
                turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees right
                // and go backwards around 12 sqrt 2 inches to be
                // around the center of the crater, could turn another
                // 90 degrees right to be facing the rover
            }

        }

        if (blockLocation.equals("CENTER")){
            goForward(59.397);
            turn(45.0);
            goForward(12.0);
            turn(-90.0);
            depositMarker();
            turn(180);
            goForward(48.0);

            if (blockLocation.equals("LEFT")){
                turn(90);
                goForward(12.0);
                turn(-90.0);
                goForward(36.0);
                // You are on the crater wall
                //You could turn around a little more than
                // 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

            else if (blockLocation.equals("RIGHT")){
                turn(90);
                goForward(36.0);
                turn(-90.0);
                goForward(60.0);
                // You are on the crater wall
                //You could turn around a little more than
                // 45 degrees right or something and go like
                //12 sqrt 2 inches to be around the center of the
                // crater, could turn another 90 degrees right to be
                // facing the rover
            }

            else if (blockLocation.equals("CENTER")){
                turn(90.0);
                goForward(24.0);
                turn(-90.0);
                goForward(48.0);
                // You are on the crater wall
                //You could turn a little less than 45 degrees
                // right and go backwards around 12 sqrt 2 inches
                // to be around the center of the crater,
                // could turn another 90 degrees right to be facing the rover
            }

        }


    }

    // starts 75 inches from the wall (flush with the line)
    public void knockFirstBlock() throws InterruptedException {
        if (blockLocation.equals("CENTER")) {
            moveToDistance(24);

        } else if (blockLocation.equals("RIGHT")) {
            turn(-45);

            moveToDistance(6);

            turn(90);

            moveToDistance(12);

            depositMarker();


        } else if (blockLocation.equals("LEFT")) {
            goForward(25.4558);
            turn(45.0);
            goForward(36);
            turn(90.0);
            goForward(24);
            depositMarker();
            turn(180.0);
            goForward(48.0);
        }
    }

    public void goToSecondBlocks() {
        while(getDistB() < 96 && opModeIsActive()) {
            driveBackward();
        }

        stopDriveMotors();
        turn(-90);

        while(getDistB() > 36) {
            driveForward();
        }
        stopDriveMotors();

    }

    public void knockSecondBlock() {
        if(blockLocation.equals("CENTER")) {
            while (getDistB() > 36 && opModeIsActive()) {
                driveForward();
            }
            stopDriveMotors();
        } else if(blockLocation.equals("RIGHT")){

        } else if(blockLocation.equals("LEFT")) {

        }
    }
}