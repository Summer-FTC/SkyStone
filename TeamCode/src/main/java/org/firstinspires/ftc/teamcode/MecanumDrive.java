package org.firstinspires.ftc.teamcode;

public class MecanumDrive
{

    // Converts degree parameter (0 - 360) to radians
    public double convertToRadians (double degrees)
    {
        double rad = degrees/180;
        return rad;
    }

    // Vd = Desired robot speed [−1,1]
    // angle = Desired robot angle [0, 360]
    // Vt = Desired speed for changing direction [−1,1]
    public double WheelSpeed (double Vd, double angle, double Vt, String wheel)
    {
        double rad = convertToRadians(angle);
        double v = 0.0; // wheel speed

        // Front right and back left : V = Vd * Math.cos(Td + (Math.PI / 4)) + Vt;
        if (wheel.equals("FR") || wheel.equals("BL"))
        {
            v = Vd * Math.cos(rad + (Math.PI / 4)) + Vt;
        }

        // Front left and back right : V = Vd * Math.sin(Td + (Math.PI / 4)) + Vt;
        else if(wheel.equals("FL") || wheel.equals("BR"))
        {
            v = Vd * Math.sin(rad + (Math.PI / 4)) + Vt;
        }

        return v;
    }


}


// https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
// ^^ might need to add multipliers as seen here