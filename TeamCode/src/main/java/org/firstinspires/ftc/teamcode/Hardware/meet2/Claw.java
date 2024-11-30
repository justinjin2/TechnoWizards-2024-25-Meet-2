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
    public static final int CLOSE_SERVO = 0;
    public static final int OPEN_SERVO = 1;

    public double clawOffset = CLOSE_SERVO;
    public double servo1Offset = MID_SERVO;
    public double servo2Offset = MID_SERVO;

    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        servo1 = hwMap.get(ServoImplEx.class, "servo1");
        servo2 = hwMap.get(ServoImplEx.class, "servo2");
        claw = hwMap.get(ServoImplEx.class, "claw");

        servo1.setPosition(servo1Offset);
        servo2.setPosition(servo2Offset);
        claw.setPosition(CLOSE_SERVO);
    }

    public void wristForward(double incremental){
        servo1Offset -= incremental;
        servo2Offset += incremental;
        servo1.setPosition(servo1Offset);
        servo2.setPosition(servo2Offset);
    }

    public void wristBackward(double incremental){
        servo1Offset += incremental;
        servo2Offset -= incremental;
        servo1.setPosition(servo1Offset);
        servo2.setPosition(servo2Offset);
    }

    public void wristGoCenter(){
        servo1Offset = MID_SERVO;
        servo2Offset = MID_SERVO;
        servo1.setPosition(MID_SERVO);
        servo2.setPosition(MID_SERVO);
    }
    public void wristLeft(double incremental){
        servo1Offset -= incremental;
        servo2Offset -= incremental;
        servo1.setPosition(servo1Offset);
        servo2.setPosition(servo2Offset);
    }
    public void wristRight(double incremental) {
        servo1Offset += incremental;
        servo2Offset += incremental;
        servo1.setPosition(servo1Offset);
        servo2.setPosition(servo2Offset);
    }
    public void clawClose(){
        claw.setPosition(CLOSE_SERVO);
    }
    public void clawOpen(){
        claw.setPosition(OPEN_SERVO);
    }


}
