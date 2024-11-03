package org.firstinspires.ftc.teamcode.telestuff;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.robot;

@TeleOp(name="teletesting")
public class tele extends LinearOpMode{

    robot robot = new robot();

    double SpeedAdjust = 1;

    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);
        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.5);

        waitForStart();


        if (isStopRequested()) return;

        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.5);

        while (opModeIsActive() && !isStopRequested()) {
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            robot.leftBack.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            robot.rightBack.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);




            if (gamepad1.left_bumper) {
                SpeedAdjust = 4;
            } else if (gamepad1.right_bumper) {
                SpeedAdjust = 1;
            }

            if (gamepad1.a) {
                robot.armMotor.setTargetPosition(200);

            }

            if (gamepad1.y) {
                robot.wrist.setPosition(0.512);
            }

            if (gamepad1.x) {//left
                robot.wrist.setPosition(0.85);
            }

            if (gamepad1.b) {//right
                robot.wrist.setPosition(0.12);
            }


            if (gamepad1.left_trigger >=0.5) {//intake
                robot.intake.setPosition(0);
            } else if (gamepad1.right_trigger >=0.5) { //outtake
                robot.intake.setPosition(1);
            }
           if (gamepad1.right_trigger ==0 && gamepad1.left_trigger == 0) {
                robot.intake.setPosition(0.5);
            }

            if (gamepad1.dpad_up) {
                robot.armMotor.setTargetPosition(2700);
            } else if (gamepad1.dpad_down) { //intakepos
                robot.armMotor.setTargetPosition(4755);
            }

            //  maxvalue:4670
//            if (gamepad1.cross) {
//                robot.armMotor.setTargetPosition();
//            }
//
//            if (gamepad1.cross) {
//                robot.wrist.setPosition(0.6);
//            }
            robot.armMotor.setPower(1);
            telemetry.addData("Desired Position", robot.armMotor.getTargetPosition());
            telemetry.addData("encoderpos:",robot.armMotor.getCurrentPosition());
            telemetry.update();


        }

    }

}
