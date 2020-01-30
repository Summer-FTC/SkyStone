package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

public enum StonePosition {
    Left(BaseLinearOpMode.MIDDLE_OF_LEFT_BLOCK_TO_LEFT_EDGE, Color.RED, 1, 3),
    Middle(BaseLinearOpMode.MIDDLE_OF_MIDDLE_BLOCK_TO_LEFT_EDGE, Color.BLUE, 2, 2),
    Right(BaseLinearOpMode.MIDDLE_OF_RIGHT_BLOCK_TO_LEFT_EDGE, Color.GREEN, 3, 1);


    private final int centerX;
    private final int centerY;
    private final int color;
    private final int bluePos;
    private final int redPos;


    StonePosition(int centerX, int color, int bluePos, int redPos) {
        this.centerX = centerX;
        this.centerY = BaseLinearOpMode.MIDDLE_OF_BLOCK_TO_TOP_OF_IMAGE;
        this.color = color;
        this.bluePos = bluePos;
        this.redPos = redPos;
    }

    public int getCenterX() {
        return centerX;
    }

    public int getCenterY() {
        return centerY;
    }

    public int getColor(){
        return color;
    }

    public int getPosition(boolean isBlue){
        if(isBlue){
            return bluePos;
        }
        else{
            return redPos;
        }
    }
}


