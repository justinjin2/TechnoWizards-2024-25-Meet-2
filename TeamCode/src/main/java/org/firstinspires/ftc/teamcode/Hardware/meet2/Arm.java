package org.firstinspires.ftc.teamcode.Hardware.meet2;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {
    public int minimumPivot = 0;
    public int maximumPivotBucket = 2500;
    public int maximumPivot = 2650;
    public int resetPivot = 2350;
    public int maximumPivotSpecimen = 3345;
    public int groundIntakePivotReady = 460 ;
    public int groundIntakePivot = 345;
    public int wallIntakePivot = 685;
    public int groundIntakeEndPivot = 540;
    public int minimumExtension = 0;
    public int groundIntakeExtension = 250;
    public int maximumIntakeExtension = 1100;
    public int maximumDeliveryExtension = 2170;
    public int specimenDeliverExtension = -130;
    public int incremental = 20;
    public int velocity = 2000;
    public int motorPower = 1;
    public double incrementalJoystickExtension = 40;
    public double incrementalJoystickPivot = 7.1;

    HardwareMap hwMap = null;

    public DcMotorEx pivotMotor;
    public DcMotorEx extensionMotor;
    public ColorRangeSensor specimenColorSensor;

    public ElapsedTime intakeTimer;

    public DigitalChannel extensionTouch = null;
    public DigitalChannel pivotTouch = null;

    Claw claw = new Claw();

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        pivotMotor = hwMap.get(DcMotorEx.class, "pivotMotor");
        extensionMotor = hwMap.get(DcMotorEx.class, "extensionMotor");
        extensionTouch = hwMap.get(DigitalChannel.class, "extensionTouch");
        pivotTouch = hwMap.get(DigitalChannel.class, "pivotTouch");
        specimenColorSensor = hwMap.get(ColorRangeSensor.class, "specimenColorSensor");

        pivotMotor.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor.setDirection(DcMotorEx.Direction.FORWARD);

        pivotMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extensionMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pivotMotor.setPower(0);
        extensionMotor.setPower(0);

        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        pivotMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        claw.init(hwMap);
    }
    public void initRunExtMotor(int power) {
        extensionMotor.setTargetPosition(0);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }
    public void initRunPivotMotor(int power) {
        pivotMotor.setTargetPosition(0);
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(power);
    }
    public void movePivotMotor(int position, int velocity){
        pivotMotor.setTargetPosition(position);
        pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotMotor.setPower(velocity);
    }
    public void moveExtensionMotor(int position, double velocity){
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(velocity);
    }
    public void moveIncrementPivotMotor(int incremental, int power){
        int currentPosition = pivotMotor.getCurrentPosition();
        int position = currentPosition + incremental;
        if (position < minimumPivot){
            pivotMotor.setTargetPosition(minimumPivot);
        } else if (position > 10000){
            pivotMotor.setTargetPosition(maximumPivot);
        } else {
            pivotMotor.setTargetPosition(position);
        }
            pivotMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(power);

    }
    public void moveIncrementExtensionMotor(int incremental, int power){
        int currentPosition = extensionMotor.getCurrentPosition();
        int position = currentPosition + incremental;
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(power);
    }
    public void moveExtensionJoystickOut(double incrementalJoystickExtension, int power){
        int currentPosition = extensionMotor.getCurrentPosition();
        double position = currentPosition + incrementalJoystickExtension;
        if (position < maximumIntakeExtension){
            extensionMotor.setTargetPosition((int) position);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(power);
        }
    }
    public void moveExtensionJoystickIn(double incrementalJoystickExtension, int power){
        int currentPosition = extensionMotor.getCurrentPosition();
        double position = currentPosition - incrementalJoystickExtension;
        if (position > minimumExtension){
            extensionMotor.setTargetPosition((int) position);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(power);
        }
    }
    public void movePivotJoystickUp(double incrementalJoystickPivot, int power){
        int currentPosition = pivotMotor.getCurrentPosition();
        double position = currentPosition + incrementalJoystickPivot;
        if (position < 570){
            pivotMotor.setTargetPosition((int) position);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(power);
        }
    }
    public void movePivotJoystickDown(double incrementalJoystick, int power){
        int currentPosition = pivotMotor.getCurrentPosition();
        double position = currentPosition - incrementalJoystick;
        if (position > groundIntakePivotReady){
            pivotMotor.setTargetPosition((int) position);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(power);
        }
    }
    public void resetMotor() {
        pivotMotor.setPower(0);
        extensionMotor.setPower(0);
        pivotMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void moveExtensionTouch(){
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTimer.reset();
            while ((intakeTimer.milliseconds() < 500)){
                extensionMotor.setPower(-0.5);
        }
    }
    public void movePivotTouch(int position, double velocity){
        extensionMotor.setTargetPosition(position);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(velocity);
    }
    public void resetTouch(){
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeTimer.reset();
        while (extensionTouch.getState() && intakeTimer.milliseconds() < 1500) {
            extensionMotor.setPower(-1);
        }
        intakeTimer.reset();
        while (intakeTimer.milliseconds() < 150) {
            claw.wristUp();
        }
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTimer.reset();
        while (pivotTouch.getState() && intakeTimer.milliseconds() < 2000){
            moveExtensionMotor(70, 0.5);
            pivotMotor.setPower(-0.7);
        }
        extensionMotor.setPower(0);
        pivotMotor.setPower(0);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        initRunPivotMotor(motorPower);
        initRunExtMotor(motorPower);
    }
    public double getSpecimenColorSensor() { return specimenColorSensor.getDistance(DistanceUnit.MM); }
}

