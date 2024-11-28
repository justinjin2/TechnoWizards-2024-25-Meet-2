package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;

@TeleOp(name="ClawTest", group="Test")
@Disabled
public class ClawTest extends LinearOpMode {
    Claw claw = new Claw();
    double SERVO_INCREMENT = 0.02;
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        claw.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry.addData("Claw position",  "Offset = %.2f", claw.clawOffset);
        telemetry.addData("Servo 1 position",  "Offset = %.2f", claw.servo1Offset);
        telemetry.addData("Servo 2 position",  "Offset = %.2f", claw.servo2Offset);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                claw.clawClose();
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                claw.clawOpen();
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                claw.wristForward(SERVO_INCREMENT);
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                claw.wristBackward(SERVO_INCREMENT);
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                claw.wristLeft(SERVO_INCREMENT);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                claw.wristRight(SERVO_INCREMENT);
            }


            telemetry.addData("Claw position",  "Offset = %.2f", claw.clawOffset);
            telemetry.addData("Servo 1 position",  "Offset = %.2f", claw.servo1Offset);
            telemetry.addData("Servo 2 position",  "Offset = %.2f", claw.servo2Offset);
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            sleep(50);
        }
    }
}