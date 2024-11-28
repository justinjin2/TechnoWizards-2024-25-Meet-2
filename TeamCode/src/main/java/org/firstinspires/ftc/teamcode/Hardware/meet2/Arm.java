package org.firstinspires.ftc.teamcode.Hardware.meet2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    int minimumPivot = 0;
    int maximumPivot = 1865;
    int minimumExtension = 0;
    int maximumExtension = 1865;
    public int incremental = 28;
    public int velocity = 2000;
    HardwareMap hwMap = null;

    public DcMotorEx pivotMotor;
    public DcMotorEx extensionMotor;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        pivotMotor = hwMap.get(DcMotorEx.class, "pivotMotor");
        extensionMotor = hwMap.get(DcMotorEx.class, "extensionMotor");

        pivotMotor.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor.setDirection(DcMotorEx.Direction.FORWARD);

        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pivotMotor.setPower(0);
        extensionMotor.setPower(0);

        pivotMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
    public void movePivotMotor(int position, int velocity){
        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setVelocity(velocity);
    }
    public void moveExtensionMotor(int position, int velocity){
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor.setVelocity(velocity);
    }
    public void moveIncrementPivotMotor(int incremental, int velocity){
        int currentPosition = pivotMotor.getCurrentPosition();
        int position = currentPosition + incremental;
        if (position < minimumPivot){
            pivotMotor.setTargetPosition(minimumPivot);
        } else if (position > maximumPivot){
            pivotMotor.setTargetPosition(maximumPivot);
        } else {
            pivotMotor.setTargetPosition(position);
        }
            pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            pivotMotor.setVelocity(velocity);

    }
    public void moveIncrementExtensionMotor(int incremental, int velocity){
        int currentPosition = extensionMotor.getCurrentPosition();
        int position = currentPosition + incremental;
        if (position < minimumExtension){
            extensionMotor.setTargetPosition(minimumExtension);
        } else if (position > maximumExtension){
            extensionMotor.setTargetPosition(maximumExtension);
        } else {
            extensionMotor.setTargetPosition(position);
        }
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setVelocity(velocity);
    }
}
