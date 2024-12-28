package org.firstinspires.ftc.teamcode.Hardware.meet2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Claw {
    HardwareMap hwMap = null;

    public ServoImplEx servo1 = null;
    public ServoImplEx servo2 = null;
    public ServoImplEx claw = null;
    public static final double MID_SERVO = 0.57;
    public static final double CLOSE_SERVO = 0.28;
    public static final double OPEN_SERVO = 0.51;
    public static final double WRIST_UP = 0.97;
    public static final double WRIST_DOWN = 0.23;
    public static final double WRIST_DELIVER = 0.8;
    public static final double SPECIMEN_DELIVER_SERVO = 0.23;
    public static final double SPECIMEN_READY_SERVO = 0.93;
    public static final double joystickIncrement = 0.001;
    public static final double differentialMax = 0.31;
    public static final double differentialMin = 0.15;

    public ElapsedTime clawTimer;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        servo1 = hwMap.get(ServoImplEx.class, "servo1");
        servo2 = hwMap.get(ServoImplEx.class, "servo2");
        claw = hwMap.get(ServoImplEx.class, "claw");
    }

    public void wristForward(double incremental){
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 += incremental;
        position2 += incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }

    public void wristBackward(double incremental){
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 -= incremental;
        position2 -= incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }

    public void wristUp(){
        servo1.setPosition(WRIST_UP);
        servo2.setPosition(WRIST_UP);
    }

    public void wristDown(){
        servo1.setPosition(WRIST_DOWN);
        servo2.setPosition(WRIST_DOWN);
    }

    public void wristCenter(){
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);
    }
    public void wristRight(double incremental){
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 += incremental;
        position2 -= incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }
    public void wristLeft(double incremental) {
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 -= incremental;
        position2 += incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }
    public void wristRightJoystick(double wristIncremental) {
        double currentPosition1 = servo1.getPosition();
        double currentPosition2 = servo2.getPosition();
        double position1 = currentPosition1 + wristIncremental;
        double position2 = currentPosition2 - wristIncremental;
        if (position1 <= differentialMax && position2 >= differentialMin) {
            servo1.setPosition(position1);
            servo2.setPosition(position2);
        }
    }
    public void wristLeftJoystick(double wristIncremental) {
        double currentPosition1 = servo1.getPosition();
        double currentPosition2 = servo2.getPosition();
        double position1 = currentPosition1 - wristIncremental;
        double position2 = currentPosition2 + wristIncremental;
        if (position1 >= differentialMin && position2 <= differentialMax) {
            servo1.setPosition(position1);
            servo2.setPosition(position2);
        }
    }
    public void wristDeliver(){
        servo1.setPosition(WRIST_DELIVER);
        servo2.setPosition(WRIST_DELIVER);
    }
    public void wristReadySpecimen(){
        servo1.setPosition(SPECIMEN_READY_SERVO);
        servo2.setPosition(SPECIMEN_READY_SERVO);
    }
    public void wristDeliverSpecimen(){
        servo1.setPosition(SPECIMEN_DELIVER_SERVO);
        servo2.setPosition(SPECIMEN_DELIVER_SERVO);
    }
    public void clawClose(){
        claw.setPosition(CLOSE_SERVO);
    }
    public void clawCenter() {claw.setPosition(0.5);}
    public void clawOpen(){
        claw.setPosition(OPEN_SERVO);
    }
    public void incrementClaw(double incremental) {
        double position = claw.getPosition();
        position += incremental;
        claw.setPosition(position);
    }


}
