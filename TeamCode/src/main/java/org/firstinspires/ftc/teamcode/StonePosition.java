package org.firstinspires.ftc.teamcode;



public enum StonePosition {
    Left(BaseLinearOpMode.MIDDLE_OF_LEFT_BLOCK_TO_LEFT_EDGE),
    Middle(BaseLinearOpMode.MIDDLE_OF_MIDDLE_BLOCK_TO_LEFT_EDGE),
    Right(BaseLinearOpMode.MIDDLE_OF_RIGHT_BLOCK_TO_LEFT_EDGE);


    private final int centerX;
    private final int centerY;

    StonePosition(int centerX) {
        this.centerX = centerX;
        this.centerY = BaseLinearOpMode.MIDDLE_OF_BLOCK_TO_TOP_OF_IMAGE;
    }

    public int getCenterX() {
        return centerX;
    }

    public int getCenterY() {
        return centerY;
    }



}
