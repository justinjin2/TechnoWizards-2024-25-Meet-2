package org.firstinspires.ftc.teamcode.Hardware.meet2;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Arm {
    public int minimumPivot = 0;
    public int maximumPivotBucket = 2500;
    public int maximumPivot = 2660;
    public int resetPivot = 2350;
    public int maximumPivotSpecimen = 3345;
    public int groundIntakePivotReady = 480 ;
    public int groundIntakePivot = 345;
    public int wallIntakePivot = 730;
    public int wallIntakePivotAuto = 687 + 20;
    public int groundIntakeEndPivot = 540;
    public int minimumExtension = 0;
    public int groundIntakeExtension = 250;
    public int maximumIntakeExtension = 1100;
    public int maximumDeliveryExtension = 1900;
    public int specimenDeliverExtension = -150;
    public int incremental = 20;
    public int velocity = 2000;
    public int motorPower = 1;
    public double incrementalJoystickExtension = 100;
    public double incrementalJoystickPivot = 19;
    public double parkServoDown = 0;
    public double parkServoUp = 0.42;


    HardwareMap hwMap = null;

    public DcMotorEx pivotMotor;
    public DcMotorEx extensionMotor;
    public ColorRangeSensor specimenColorSensor;
    public ServoImplEx parkServo;
    public ServoImplEx leftAligner;

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
        parkServo = hwMap.get(ServoImplEx.class, "parkServo");
        leftAligner = hwMap.get(ServoImplEx.class, "alignLeft");

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
        driveAligner();
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
    public void initAligner(){
        parkServo.setPosition(0);
        leftAligner.setPosition(1);
    }
    public void driveAligner(){
        parkServo.setPosition(1);
        leftAligner.setPosition(0);
    }
    public void specimenAligner(){
        parkServo.setPosition(0.7);
        leftAligner.setPosition(0.27);
    }
    public double getSpecimenColorSensor() { return specimenColorSensor.getDistance(DistanceUnit.MM); }
}

