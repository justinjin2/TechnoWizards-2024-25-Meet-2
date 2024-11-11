
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.robot.robot;
import com.qualcomm.robotcore.hardware.DcMotor;

/** This is a subsystem, for the claw of our robot
 * Here we make methods to manipulate the servos
 * We also import RobotConstants to get the positions of the servos.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 9/8/2024
 */

public class armandintake {
    robot robot = new robot();

    int DEPOSITARMPOS = 2970;
    int PREINTAKEARMPOS = 4225;
    int DRIVEARMPOS = 1000;
    int INTAKEARMPOS = 4735;
    int SPECARMPOS = 2900;
    int AFTERSPECDROPPOS = 3200;
    double WRISTCENTERPOS = 0.512;
    double INTAKEIN = 0;
    double INTAKEOUT = 1;
    double INTAKEOFF = 0.5;
    public DcMotorEx armMotor;



    public Servo wrist;
    public Servo intake;


    /** This is the constructor for the subsystem, it maps the servos to the hardwareMap.
     * The device names should align with the configuration names on the driver hub.
     * To use this subsystem, we have to import this file, declare the subsystem (private ClawSubsystem claw;),
     * and then call the below constructor in the init() method. */

    public armandintake(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
        intake = hardwareMap.get(Servo.class, "intake");

    }

    //------------------------------Close Claws------------------------------//
    public void armspecpos() {
                armMotor.setPower(1);
                armMotor.setTargetPosition(SPECARMPOS);
               armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               wrist.setPosition(1);
    }
    public void armspecppull() {
        armMotor.setPower(1);
        armMotor.setTargetPosition(AFTERSPECDROPPOS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wrist.setPosition(1);
    }

//    public void closeRClaw() {
//        clawR.setPosition(RobotConstants.closedR);
//    }
//
//    public void closeClaws() {
//        clawL.setPosition(RobotConstants.closedL);
//        clawR.setPosition(RobotConstants.closedR);
//    }
//
//    //------------------------------Open Claws------------------------------//
//    public void openLClaw() {
//        clawL.setPosition(RobotConstants.openL);
//    }
//
//    public void openRClaw() {
//        clawR.setPosition(RobotConstants.openR);
//    }
//
//    public void openClaws() {
//        clawL.setPosition(RobotConstants.openL);
//        clawR.setPosition(RobotConstants.openR);
//    }
//
//    //------------------------------Claw Rotate------------------------------//
//
//    public void startClaw() {
//        pivot.setPosition(RobotConstants.startClaw);
//    }
//
//    public void groundClaw() {
//        pivot.setPosition(RobotConstants.groundClaw);
//    }
//
//    public void scoringClaw() {
//        pivot.setPosition(RobotConstants.scoringClaw);
//    }

}
