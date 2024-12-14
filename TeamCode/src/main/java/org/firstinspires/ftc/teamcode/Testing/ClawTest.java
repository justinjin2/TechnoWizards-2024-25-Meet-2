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
    double SERVO_INCREMENT = 0.01;
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        claw.init(hardwareMap);
        claw.clawOpen();
        claw.servo1.setPosition(0.5);
        claw.servo2.setPosition(0.5);
        telemetry.update();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Claw position",  "Offset = %.2f", claw.claw.getPosition());
        telemetry.addData("Servo 1 position",  "Offset = %.2f", claw.servo1.getPosition());
        telemetry.addData("Servo 2 position",  "Offset = %.2f", claw.servo2.getPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                claw.clawOpen();
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                claw.clawClose();
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                claw.wristUp();
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                claw.wristDown();
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                claw.wristLeft(SERVO_INCREMENT);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                claw.wristRight(SERVO_INCREMENT);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                claw.wristCenter();
            }

            telemetry.addData("Claw position",  "Offset = %.2f", claw.claw.getPosition());
            telemetry.addData("Servo 1 position",  "Offset = %.2f", claw.servo1.getPosition());
            telemetry.addData("Servo 2 position",  "Offset = %.2f", claw.servo2.getPosition());
            telemetry.update();
            previousGamepad1.copy(currentGamepad1);
            sleep(50);
        }
    }
}