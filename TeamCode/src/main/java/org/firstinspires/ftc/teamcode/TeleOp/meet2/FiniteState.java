package org.firstinspires.ftc.teamcode.TeleOp.meet2;

public enum FiniteState{
    IDLE,
    INTAKE_WALL_START,
    INTAKE_GROUND_PIVOT,
    INTAKE_GROUND_CLAW,
    INTAKE_WALL_END,
    INTAKE_GROUND_END,
    DELIVERY_HIGH_BUCKET_PIVOT,
    DELIVERY_HIGH_BUCKET,
    DELIVERY_LOW_BUCKET,
    DELIVERY_OPEN,
    DELIVERY_SPECIMEN_START,
    DELIVERY_SPECIMEN,
    EXTENSION_RESET_BUCKET,
    PIVOT_RESET_BUCKET,
    EXTENSION_RESET_SPECIMEN,
    PIVOT_RESET_SPECIMEN,
    RESET_POSITION_BUCKET,
    RESET_POSITION_SPECIMEN,
    HANG_READY,
    HANG_END,
    // Used for Auto
    SCORE_PRELOAD,
    PIVOT_READY,
    CLIP_DELIVERY_READY,
    DELIVERY_DONE,
    SAMPLE_PUSH,
    INTAKE_WALL_PRE_END,
    DECISION,
    SAMPLE_PUSH_DONE
}
