package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;

@TeleOp(name="MotorTest", group="Test")
@Disabled
public class MotorTest extends LinearOpMode {
Arm arm = new Arm();
    // Declare OpMode members.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        arm.init(hardwareMap);
        arm.resetMotor();
        arm.initRunExtMotor(arm.motorPower);
//        arm.initRunPivotMotor(arm.motorPower);
        waitForStart();

        arm.movePivotMotor(arm.wallIntakePivot, arm.motorPower);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            currentGamepad1.copy(gamepad1);
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                arm.moveIncrementPivotMotor(arm.incremental, arm.motorPower);
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                arm.moveIncrementPivotMotor(-arm.incremental, arm.motorPower);
            }
            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
                arm.moveIncrementExtensionMotor(arm.incremental, arm.motorPower);
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left){
                arm.moveIncrementExtensionMotor(-arm.incremental, arm.motorPower);
            }
            if (currentGamepad1.a && !previousGamepad1.a){
                arm.moveExtensionMotor(arm.maximumIntakeExtension, arm.motorPower);
            }
            if (currentGamepad1.b && !previousGamepad1.b){
                arm.movePivotMotor(arm.maximumPivot, arm.motorPower);
                sleep(500);
                arm.moveExtensionMotor(arm.maximumDeliveryExtension, arm.motorPower);
            }
            if (gamepad1.left_stick_y < 0){
                arm.moveExtensionJoystickOut(arm.incrementalJoystickExtension, arm.motorPower);
                arm.movePivotJoystickDown(arm.incrementalJoystickPivot, arm.motorPower);
            }
            if (gamepad1.left_stick_y > 0){
                arm.moveExtensionJoystickIn(arm.incrementalJoystickExtension, arm.motorPower);
                arm.movePivotJoystickUp(arm.incrementalJoystickPivot, arm.motorPower);
            }

            previousGamepad1.copy(currentGamepad1);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Arm Motor Position", arm.pivotMotor.getCurrentPosition());
            telemetry.addData("Extension Motor Position", arm.extensionMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}

