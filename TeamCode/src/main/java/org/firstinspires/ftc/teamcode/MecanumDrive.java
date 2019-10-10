package org.firstinspires.ftc.teamcode;

public class MecanumDrive
{

    // Converts degree parameter (0 - 360) to radians
    public static double convertToRadians(double degrees) {
        double rad = degrees / 180;
        return rad;
    }

    // Vd = Desired robot speed [−1,1]
    // angle = Desired robot angle [0, 360]
    // Vt = Desired speed for changing direction [−1,1]
    public static double wheelSpeed(double Vd, double angle, double Vt, String wheel)
    {
        double rad = convertToRadians(angle);
        double v = 0.0; // wheel speed;

        if (wheel.equals("FR") || wheel.equals("BL"))
        {
            v = Vd * Math.cos(rad + (Math.PI / 4)) + Vt;
        }

        else if (wheel.equals("FL") || wheel.equals("BR"))
        {
            v = Vd * Math.sin(rad + (Math.PI / 4)) + Vt;
        }

        if (v > 1)
        {
            v = 1;
        }

        return v;
    }

}


// https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
// ^^ might need to add multipliers as seen here