package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;

@TeleOp(name="HardStopTest", group="Test")
//@Disabled
public class HardStopTest extends LinearOpMode {
    Claw claw = new Claw();
    Arm arm = new Arm();
    double SERVO_INCREMENT = 0.01;
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        claw.init(hardwareMap);
        arm.init(hardwareMap);
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        arm.initAligner();

        waitForStart();

        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.x && !previousGamepad1.x) {
                arm.parkServo.setPosition(0);
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                arm.parkServo.setPosition(1);
            }

            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                arm.leftAligner.setPosition(0);
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                arm.leftAligner.setPosition(1);
            }

            if (currentGamepad1.y && !previousGamepad1.y) {
                arm.parkServo.setPosition(arm.parkServo.getPosition() + SERVO_INCREMENT);
            }

            if (currentGamepad1.a && !previousGamepad1.a) {
                arm.parkServo.setPosition(arm.parkServo.getPosition() - SERVO_INCREMENT);
            }

            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                arm.driveAligner();
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                arm.specimenAligner();
            }

            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                arm.initAligner();
            }

            telemetry.addData("Park Servo Position", arm.parkServo.getPosition());
            telemetry.addData("Left Aligner Position", arm.leftAligner.getPosition());
            telemetry.update();

            previousGamepad1.copy(currentGamepad1);
        }
    }
}