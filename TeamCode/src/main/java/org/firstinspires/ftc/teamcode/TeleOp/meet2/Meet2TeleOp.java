package org.firstinspires.ftc.teamcode.TeleOp.meet2;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;
import org.firstinspires.ftc.teamcode.Hardware.meet2.DriveTrain;

@TeleOp(name = "Meet2TeleOp", group = "Test")
public class Meet2TeleOp extends LinearOpMode {
    Arm arm = new Arm();
    Claw claw = new Claw();
    DriveTrain driveTrain = new DriveTrain();

    private int INTAKE_CHECK;
    private int DELIVERY_CHECK;

    private FiniteState finiteState = FiniteState.IDLE;
    ElapsedTime loopTimer;
    ElapsedTime waitingTimer;

    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        driveTrain.init(hardwareMap);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        loopTimer = new ElapsedTime();
        waitingTimer = new ElapsedTime();
        arm.intakeTimer = new ElapsedTime();
        claw.clawTimer = new ElapsedTime();

        claw.wristUp();
        claw.clawClose();
        arm.initRunExtMotor(arm.motorPower);
        arm.initRunPivotMotor(arm.motorPower);

        waitForStart();

        if (isStopRequested()) return;

        while (arm.extensionTouch.getState() && arm.pivotTouch.getState()){
            arm.resetTouch();
            sleep(500);
        }

        arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);

        while (opModeIsActive() && !isStopRequested()) {

            loopTimer.reset();

            if (gamepad1.left_trigger > 0) {
                driveTrain.leftSpeedAdjust = 0.7;
            } else {
                driveTrain.leftSpeedAdjust = 1;
            }

            if (gamepad1.right_trigger > 0) {
                driveTrain.rightSpeedAdjust = 0.5;
            } else {
                driveTrain.rightSpeedAdjust = 1;
            }

            driveTrain.Speed = -gamepad1.left_stick_y;
            driveTrain.Strafe = gamepad1.left_stick_x;
            driveTrain.Turn = -gamepad1.right_stick_x;

            driveTrain.leftFrontPower = Range.clip(driveTrain.Speed + driveTrain.Strafe + driveTrain.Turn, -1, 1);
            driveTrain.rightFrontPower = Range.clip(driveTrain.Speed - driveTrain.Strafe - driveTrain.Turn, -1, 1);
            driveTrain.leftRearPower = Range.clip(driveTrain.Speed - driveTrain.Strafe + driveTrain.Turn, -1, 1);
            driveTrain.rightRearPower = Range.clip(driveTrain.Speed + driveTrain.Strafe - driveTrain.Turn, -1, 1);

            driveTrain.leftFront.setPower(driveTrain.leftFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightFront.setPower(driveTrain.rightFrontPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.leftRear.setPower(driveTrain.leftRearPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);
            driveTrain.rightRear.setPower(driveTrain.rightRearPower * driveTrain.rightSpeedAdjust * driveTrain.leftSpeedAdjust);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (currentGamepad2.a && !previousGamepad2.a) { //Ground Intake
                arm.movePivotMotor(arm.groundIntakePivotReady, arm.motorPower);
                arm.moveExtensionMotor(arm.groundIntakeExtension, arm.motorPower);
                claw.clawOpen();
                claw.wristDown();
                finiteState = FiniteState.INTAKE_GROUND_PIVOT;
            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_down) {
                INTAKE_CHECK = 0;
                arm.movePivotMotor(arm.maximumPivotBucket, arm.motorPower);
                claw.wristDeliver();
                finiteState = FiniteState.DELIVERY_HIGH_BUCKET_PIVOT;
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                INTAKE_CHECK = 0;
                arm.movePivotMotor(arm.maximumPivotBucket, arm.motorPower);
                arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                claw.wristDeliver();
                finiteState = FiniteState.DELIVERY_LOW_BUCKET;
            }
            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                if (DELIVERY_CHECK == 1) {
                    claw.clawOpen();
                    finiteState = FiniteState.DELIVERY_OPEN;
                    claw.clawTimer.reset();
                } else if (DELIVERY_CHECK == 2) {
                    claw.wristDeliverSpecimen();
                    INTAKE_CHECK = 0;
                    finiteState = FiniteState.DELIVERY_SPECIMEN;
                    claw.clawTimer.reset();
                }
            }
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                if (INTAKE_CHECK == 1) {
                    DELIVERY_CHECK = 1;
                    arm.movePivotMotor(arm.groundIntakePivot, arm.motorPower);
                    finiteState = FiniteState.INTAKE_GROUND_CLAW;
                    arm.intakeTimer.reset();
                } else if (INTAKE_CHECK == 2) {
                    DELIVERY_CHECK = 2;
                    claw.clawClose();
                    finiteState = FiniteState.INTAKE_WALL_END;
                    arm.intakeTimer.reset();
                }
            }

            if (currentGamepad2.b && !previousGamepad2.b) {
                claw.wristCenter();
                claw.clawOpen();
                arm.movePivotMotor(arm.wallIntakePivot, arm.motorPower);
                arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                finiteState = FiniteState.INTAKE_WALL_START;
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                arm.movePivotMotor(arm.maximumPivot, arm.motorPower);
            }

            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                arm.movePivotMotor(arm.minimumPivot, arm.motorPower);
            }

            if (gamepad2.left_stick_y < 0) {
                arm.moveExtensionJoystickOut(arm.incrementalJoystickExtension, arm.motorPower);
                if (arm.extensionMotor.getCurrentPosition() < arm.maximumIntakeExtension) {
                    arm.movePivotJoystickUp(arm.incrementalJoystickPivot, arm.motorPower);
                }
            }
            if (gamepad2.left_stick_y > 0) {
                arm.moveExtensionJoystickIn(arm.incrementalJoystickExtension, arm.motorPower);
                arm.movePivotJoystickDown(arm.incrementalJoystickPivot, arm.motorPower);
            }
            if (gamepad2.right_stick_x < 0) {
                claw.wristRightJoystick(claw.joystickIncrement);
            }
            if (gamepad2.right_stick_x > 0) {
                claw.wristLeftJoystick(claw.joystickIncrement);
            }
            if (currentGamepad2.back && !previousGamepad2.back) {
                arm.resetTouch();
                sleep(500);
                arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);
            }

            switch (finiteState) {
                case INTAKE_GROUND_PIVOT: {
                    if (arm.pivotMotor.getCurrentPosition() + 15 >= arm.groundIntakePivotReady) {
                        INTAKE_CHECK = 1;
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case INTAKE_GROUND_CLAW: {
                    if (arm.intakeTimer.milliseconds() > 150) {
                        claw.clawClose();
                        finiteState = FiniteState.INTAKE_GROUND_END;
                        arm.intakeTimer.reset();
                    }
                    break;
                }
                case INTAKE_GROUND_END: {
                    if (arm.intakeTimer.milliseconds() > 100) {
                        arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                        arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);
                        claw.wristDown();
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case INTAKE_WALL_START: {
                    INTAKE_CHECK = 2;
                    if (arm.getSpecimenColorSensor() <= 20) {
                        DELIVERY_CHECK = 2;
                        claw.clawClose();
                        finiteState = FiniteState.INTAKE_WALL_END;
                        arm.intakeTimer.reset();
                    }
                    break;
                }
                case INTAKE_WALL_END: {
                    if (arm.intakeTimer.milliseconds() > 100) {
                        claw.wristReadySpecimen();
                        arm.movePivotMotor(arm.maximumPivot, arm.motorPower);
                        arm.moveExtensionMotor(arm.specimenDeliverExtension, arm.motorPower);
                        finiteState = FiniteState.DELIVERY_SPECIMEN_START;
                    }
                    break;
                }
            }


            switch (finiteState) {
                case DELIVERY_HIGH_BUCKET_PIVOT: {
                    if (arm.pivotMotor.getCurrentPosition() + 200 >= arm.maximumPivotBucket) {
                        arm.moveExtensionMotor(arm.maximumDeliveryExtension, arm.motorPower);
                        finiteState = FiniteState.DELIVERY_HIGH_BUCKET;
                    }
                    break;
                }
                case DELIVERY_HIGH_BUCKET: {
                    if (arm.extensionMotor.getCurrentPosition() + 15 >= arm.maximumDeliveryExtension) {
                        DELIVERY_CHECK = 1;
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case DELIVERY_LOW_BUCKET: {
                    if ((arm.pivotMotor.getCurrentPosition() + 15 >= arm.maximumPivotBucket)
                            && (arm.extensionMotor.getCurrentPosition() + 15 >= arm.minimumExtension)) {
                        DELIVERY_CHECK = 1;
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case DELIVERY_OPEN: {
                    if (claw.clawTimer.milliseconds() > 100) {
                        claw.wristDown();
                        finiteState = FiniteState.EXTENSION_RESET_BUCKET;
                        claw.clawTimer.reset();
                    }
                    break;
                }
                case DELIVERY_SPECIMEN_START: {
                    if (arm.pivotMotor.getCurrentPosition() + 15 >= arm.maximumPivotSpecimen) {
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case DELIVERY_SPECIMEN: {
                    if (claw.clawTimer.milliseconds() > 350) {
                        claw.clawOpen();
                        finiteState = FiniteState.PIVOT_RESET_SPECIMEN;
                        claw.clawTimer.reset();
                    }
                    break;
                }
                case EXTENSION_RESET_BUCKET: {
                    if (claw.clawTimer.milliseconds() > 200) {
                        arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                        finiteState = FiniteState.PIVOT_RESET_BUCKET;
                    }
                    break;
                }
                case PIVOT_RESET_BUCKET: {
                    if (arm.extensionMotor.getCurrentPosition() <= arm.minimumExtension + 400) {
                        arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);
                        finiteState = FiniteState.RESET_POSITION_BUCKET;
                    }
                    break;
                }
                case PIVOT_RESET_SPECIMEN: {
                    if (claw.clawTimer.milliseconds() > 300) {
                        arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);
                        finiteState = FiniteState.EXTENSION_RESET_SPECIMEN;
                    }
                    break;
                }
                case EXTENSION_RESET_SPECIMEN: {
                    if (arm.pivotMotor.getCurrentPosition() <= arm.resetPivot) {
                        arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                        finiteState = FiniteState.RESET_POSITION_SPECIMEN;
                    }
                }
                case RESET_POSITION_SPECIMEN: {
                    if (arm.extensionMotor.getCurrentPosition() + 50 >= arm.minimumExtension) {
                        claw.wristUp();
                        claw.clawClose();
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case RESET_POSITION_BUCKET: {
                    if (arm.pivotMotor.getCurrentPosition() <= arm.groundIntakeEndPivot + 50) {
                        DELIVERY_CHECK = 0;
                        claw.wristUp();
                        claw.clawClose();
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
            }

            switch (finiteState) {
                case HANG_READY: {
                    if (arm.pivotMotor.getCurrentPosition() + 30 >= arm.maximumPivot) {
                        finiteState = FiniteState.IDLE;
                    }
                    break;
                }
                case HANG_END: {
                    if (arm.pivotMotor.getCurrentPosition() <= arm.groundIntakePivot + 30) {
                        finiteState = FiniteState.IDLE;
                        ;
                    }
                    break;
                }
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                arm.moveIncrementPivotMotor(-arm.incremental, arm.motorPower);
            }
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                arm.moveIncrementPivotMotor(arm.incremental, arm.motorPower);
            }
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

//            telemetry.addData("Arm Motor Position", arm.pivotMotor.getCurrentPosition());
//            telemetry.addData("Extension Motor Position", arm.extensionMotor.getCurrentPosition());
//            telemetry.addData("Claw position", claw.claw.getPosition());
//            telemetry.addData("Servo 1 position", claw.servo1.getPosition());
//            telemetry.addData("Servo 2 position", claw.servo2.getPosition());
            telemetry.addData("Loop Timer", loopTimer.milliseconds());
            telemetry.update();
        }
    }
}
