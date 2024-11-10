package org.firstinspires.ftc.teamcode.telestuff;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.robot;
import org.firstinspires.ftc.teamcode.robot.RobotConstants;

@TeleOp(name="teletesting")
public class tele extends LinearOpMode{


    robot robot = new robot();
 //   RobotConstants constants = new RobotConstants();

    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;


    public DcMotorEx armMotor;



    public Servo wrist;
    public Servo intakeservo;

    enum intakeState { //INTAKE
       START,
        Ready,
       Intake,
        AFTERINTAKE
    }

    enum Deposit { //INTAKE
        START,
        Depositpos,
       outtake
    }

    enum Speciman { //INTAKE
        START,
        Depositpos,
        outtake,
        endofouttake
    }
    double SpeedAdjust = 1;
    HardwareMap hwMap = null;
   public void initialize() {
       leftFront = hwMap.get(DcMotorEx.class, "leftFront");
       leftBack = hwMap.get(DcMotorEx.class, "leftRear");
       rightBack = hwMap.get(DcMotorEx.class, "rightRear");
       rightFront = hwMap.get(DcMotorEx.class, "rightFront");

       armMotor = hwMap.get(DcMotorEx.class, "arm");
       wrist = hwMap.get(Servo.class, "wrist");
       intakeservo = hwMap.get(Servo.class, "intake");

       leftFront.setDirection(DcMotor.Direction.FORWARD);
       rightFront.setDirection(DcMotor.Direction.REVERSE);
       leftBack.setDirection(DcMotor.Direction.FORWARD);
       rightBack.setDirection(DcMotor.Direction.REVERSE);
       armMotor.setDirection(DcMotor.Direction.FORWARD);

       leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       leftFront.setPower(0);
       rightFront.setPower(0);
       leftBack.setPower(0);
       rightBack.setPower(0);
       armMotor.setPower(0);

       leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   }
    @Override
    public void runOpMode() {
        work();
    }

    public void work() {
        while (!isStarted() && !isStopRequested()) {
        }

        initialize();

        frameTimer.resetTimer();
        leftIntakeArm.getController().pwmEnable();
        rightIntakeArm.getController().pwmEnable();


        while (opModeIsActive()) {
            driverControlUpdate();

            telemetry();
        }
    }
    public void runOpMode() throws InterruptedException {



        armMotor.setTargetPosition(RobotConstants.STATIONARYPOS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);


        waitForStart();


        if (isStopRequested()) return;
        intakeState intake = intakeState.START;
        Deposit outtake = Deposit.START;
        Speciman specdepo = Speciman.START;

        armMotor.setTargetPosition(DRIVEARMPOS);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.9);
        wrist.setPosition(RobotConstants.WRISTCENTERPOS);

        while (opModeIsActive() && !isStopRequested()) {


            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);


            switch (intake) { // scoring pos with lift
                case START:
                    if (gamepad2.dpad_down) {
                        armMotor.setTargetPosition(PREINTAKEARMPOS);
                        wrist.setPosition(RobotConstants.WRISTCENTERPOS);

                        intake = intakeState.Intake; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                case Intake:
                    if (gamepad1.a) {
                        wrist.setPosition(RobotConstants.WRISTCENTERPOS);
                       armMotor.setTargetPosition(INTAKEARMPOS);
                        intakeservo.setPosition(RobotConstants.INTAKEON);
                        intake = intakeState.START;
                    }
                    break;



            }

            switch (outtake) { // scoring pos with lift
                case START:

                    if (gamepad2.dpad_up) {
                        wrist.setPosition(RobotConstants.WRISTCENTERPOS);
                        armMotor.setTargetPosition(DEPOSITARMPOS);
                       /// robot.intake.setPosition(0.5);
                        outtake = Deposit.outtake;
                    }
                    break;
                case outtake:
                    if (gamepad1.right_trigger >=0.5) {
                       intakeservo.setPosition(RobotConstants.INTAKEOUT);
                        outtake = Deposit.START;
                    }
                    break;
            }
            switch (specdepo) {
                case START:

                    if (gamepad1.dpad_down) {
                        armMotor.setTargetPosition(AFTERSPECDROPPOS);
                        specdepo = Speciman.START;
                    }
                    break;
            }

            if (gamepad1.y) {
                wrist.setPosition(RobotConstants.WRISTCENTERPOS);
                armMotor.setTargetPosition(PREINTAKEARMPOS);
                intakeservo.setPosition(RobotConstants.INTAKEOFF);
                intake = intakeState.START;
            }

            if (gamepad2.a) {
                wrist.setPosition(RobotConstants.WRISTCENTERPOS);
            robot.armMotor.setTargetPosition(DRIVEARMPOS);
                robot.intake.setPosition(RobotConstants.INTAKEOFF);
            }
            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 2;
            }
            if (gamepad2.b) {
                wrist.setPosition(RobotConstants.WRISTSPECPOS);
                armMotor.setTargetPosition(SPECARMPOS);
                /// robot.intake.setPosition(0.5);
            }
            if (gamepad2.left_bumper) {

                robot.armMotor.setTargetPosition(2550);
                /// robot.intake.setPosition(0.5);
            }
            if (gamepad2.right_bumper) {

                armMotor.setTargetPosition(50);
                /// robot.intake.setPosition(0.5);
            }
//
            if (gamepad1.x) {//left
                wrist.setPosition(0.99);
            }
            if (gamepad1.b) {//right
                wrist.setPosition(0.01);
            }

            robot.armMotor.setPower(1);
            telemetry.addData("Desired Position", robot.armMotor.getTargetPosition());
            telemetry.addData("Current Position:",robot.armMotor.getCurrentPosition());
            telemetry.update();
        }

    }

}
