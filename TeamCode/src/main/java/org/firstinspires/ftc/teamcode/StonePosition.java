package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

public enum StonePosition {
    Left(BaseLinearOpMode.MIDDLE_OF_LEFT_BLOCK_TO_LEFT_EDGE, Color.RED),
    Middle(BaseLinearOpMode.MIDDLE_OF_MIDDLE_BLOCK_TO_LEFT_EDGE, Color.BLUE),
    Right(BaseLinearOpMode.MIDDLE_OF_RIGHT_BLOCK_TO_LEFT_EDGE, Color.GREEN);


    private final int centerX;
    private final int centerY;
    private final int color;

    StonePosition(int centerX, int color) {
        this.centerX = centerX;
        this.centerY = BaseLinearOpMode.MIDDLE_OF_BLOCK_TO_TOP_OF_IMAGE;
        this.color = color;
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

}