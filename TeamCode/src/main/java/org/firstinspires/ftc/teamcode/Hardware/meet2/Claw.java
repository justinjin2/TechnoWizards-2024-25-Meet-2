package org.firstinspires.ftc.teamcode.Hardware.meet2;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Claw {
    HardwareMap hwMap = null;

    public ServoImplEx servo1 = null;
    public ServoImplEx servo2 = null;
    public ServoImplEx claw = null;
    public static final double MID_SERVO = 0.5;
    public static final double CLOSE_SERVO = 0.18;
    public static final double OPEN_SERVO = 0.48;
    public static final double WRIST_UP = 0.97;
    public static final double WRIST_DOWN = 0.23;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        servo1 = hwMap.get(ServoImplEx.class, "servo1");
        servo2 = hwMap.get(ServoImplEx.class, "servo2");
        claw = hwMap.get(ServoImplEx.class, "claw");
    }

    public void wristForward(double incremental, double position){
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
    public void wristLeft(double incremental){
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 += incremental;
        position2 -= incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }
    public void wristRight(double incremental) {
        double position1 = servo1.getPosition();
        double position2 = servo2.getPosition();
        position1 -= incremental;
        position2 += incremental;
        servo1.setPosition(position1);
        servo2.setPosition(position2);
    }
    public void clawClose(){
        claw.setPosition(CLOSE_SERVO);
    }
    public void clawOpen(){
        claw.setPosition(OPEN_SERVO);
    }
    public void incrementClaw(double incremental) {
        double position = claw.getPosition();
        position += incremental;
        claw.setPosition(position);
    }


}
