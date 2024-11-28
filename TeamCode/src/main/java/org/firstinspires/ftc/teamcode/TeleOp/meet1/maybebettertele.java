package org.firstinspires.ftc.teamcode.TeleOp.meet1;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.teamcode.Hardware.meet1.robot;

@TeleOp(name="nofsmtele")
public class maybebettertele extends LinearOpMode{

    private Follower follower;
    robot robot = new robot();
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
    int DEPOSITARMPOS = 2850;
    int PREINTAKEARMPOS = 4225;
    int DRIVEARMPOS = 1000;
    int INTAKEARMPOS = 4735;
    int SPECARMPOS = 2900;
    int AFTERSPECDROPPOS = 3200;
    double WRISTCENTERPOS = 0.512;
    double INTAKEIN = 0;
    double INTAKEOUT = 1;
    double INTAKEOFF = 0.5;

    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
//        robot.armMotor.setTargetPosition(DRIVEARMPOS);
//        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.armMotor.setPower(0.9);


        waitForStart();


        if (isStopRequested()) return;
        intakeState intake = intakeState.START;
        Deposit outtake = Deposit.START;
        Speciman specdepo = Speciman.START;

        robot.armMotor.setTargetPosition(DRIVEARMPOS);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.9);
        robot.wrist.setPosition(WRISTCENTERPOS);

        while (opModeIsActive() && !isStopRequested()) {

//            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
//        follower.update();
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

            if (gamepad2.dpad_down) {
                robot.armMotor.setTargetPosition(PREINTAKEARMPOS);
                robot.wrist.setPosition(WRISTCENTERPOS);

                //intake = intakeState.Intake; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
            }

                    if (gamepad1.a) {
                        robot.wrist.setPosition(WRISTCENTERPOS);
                        robot.armMotor.setTargetPosition(INTAKEARMPOS);
                        robot.intake.setPosition(INTAKEIN);

                    }


                    if (gamepad2.dpad_up) {
                        robot.wrist.setPosition(WRISTCENTERPOS);
                        robot.armMotor.setTargetPosition(DEPOSITARMPOS);
                        /// robot.intake.setPosition(0.5);

                    }

                    if (gamepad1.right_trigger >=0.5) {
                        robot.intake.setPosition(INTAKEOUT);

                    }



                    if (gamepad1.dpad_down) {
                        robot.armMotor.setTargetPosition(AFTERSPECDROPPOS);

                    }


            if (gamepad1.y) {
                robot.wrist.setPosition(WRISTCENTERPOS);
                robot.armMotor.setTargetPosition(PREINTAKEARMPOS);
                robot.intake.setPosition(INTAKEOFF);

            }

            if (gamepad2.a) {
                robot.wrist.setPosition(WRISTCENTERPOS);
                robot.armMotor.setTargetPosition(DRIVEARMPOS);
                robot.intake.setPosition(INTAKEOFF);
            }
            if (gamepad1.left_bumper) {
                SpeedAdjust = 3;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 1;
            }
            if (gamepad2.b) {
                robot.wrist.setPosition(0.01);
                robot.armMotor.setTargetPosition(SPECARMPOS);
                /// robot.intake.setPosition(0.5);
            }
            if (gamepad2.left_bumper) {

                robot.armMotor.setTargetPosition(2550);
                /// robot.intake.setPosition(0.5);
            }
            if (gamepad2.right_bumper) {
                robot.armMotor.setTargetPosition(-20);
            }
            if (gamepad1.x) {//left
                robot.wrist.setPosition(0.99);
            }
            if (gamepad1.b) {//right
                robot.wrist.setPosition(0.01);
            }
            robot.armMotor.setPower(1);
            telemetry.addData("Desired Position", robot.armMotor.getTargetPosition());
            telemetry.addData("Current Position:",robot.armMotor.getCurrentPosition());
            telemetry.update();


        }

    }

}