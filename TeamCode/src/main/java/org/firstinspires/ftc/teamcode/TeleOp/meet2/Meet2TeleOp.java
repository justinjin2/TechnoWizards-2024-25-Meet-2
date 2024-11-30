package org.firstinspires.ftc.teamcode.TeleOp.meet2;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;
import org.firstinspires.ftc.teamcode.Hardware.meet2.DriveTrain;
/**

 *

 */
@TeleOp(name = "Meet2TeleOp", group = "Test")
public class Meet2TeleOp extends LinearOpMode {
    Arm arm = new Arm();
    Claw claw = new Claw();
    DriveTrain driveTrain = new DriveTrain();
    double SpeedAdjust = 1;
    public Telemetry telemetryA;
    private FiniteState finiteState = FiniteState.INTAKE_GROUND_START;
    ElapsedTime loopTimer;
    ElapsedTime waitingTimer;
    ElapsedTime intakeTimer;
    ElapsedTime clawTimer;
    //telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
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
        intakeTimer = new ElapsedTime();
        clawTimer = new ElapsedTime();

        telemetry.addData("Robot State", "Initialized");
        /**
         * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
         * correction.
         */
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // YOUR WORK
            /*
            Define Drivetrain based on X Y keys.
             */
            loopTimer.reset();

            driveTrain.rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.leftRear.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            switch (finiteState) {
                case INTAKE_READY:{
                    if (currentGamepad1.a && !previousGamepad1.a) {
                        arm.pivotMotor.setTargetPosition(0);
                        finiteState = FiniteState.INTAKE_GROUND_READY; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                }
                case INTAKE_GROUND_READY:{
                    if (arm.pivotMotor.getCurrentPosition() >= 0 * 0.9) { //90% of desired position
                        arm.extensionMotor.setTargetPosition(0);
                        claw.servo1.setPosition(0);
                        claw.servo2.setPosition(0);
                        claw.clawOpen();
                        finiteState = FiniteState.INTAKE_GROUND;
                        intakeTimer.reset();
                    }
                    break;
                }
                case INTAKE_GROUND:{
                    if (intakeTimer.milliseconds() > 100){
                        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                        claw.clawClose();
                        finiteState = FiniteState.INTAKE_GROUND_END;
                        intakeTimer.reset();
                        }
                    }
                    break;
                }
                case INTAKE_GROUND_END:{
                    if (intakeTimer.milliseconds() > 100) {
                        claw.wristGoCenter();
                        arm.extensionMotor.setTargetPosition(0);
                        arm.pivotMotor.setTargetPosition(0);
                        finiteState = FiniteState.DELIVERY_START;
                        intakeTimer.reset();
                    }
                }
            }


            switch(finiteState){
                case DELIVERY_START:{
                    //high bucket
                    if (intakeTimer.milliseconds() > 100) {
                        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                            arm.pivotMotor.setTargetPosition(0);
                            arm.extensionMotor.setTargetPosition(0);
                            claw.servo1.setPosition(0);
                            claw.servo2.setPosition(0);
                            finiteState = FiniteState.DELIVERY_BUCKET;
                            clawTimer.reset();
                        }
                        //low bucket
                        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                            arm.pivotMotor.setTargetPosition(0);
                            arm.extensionMotor.setTargetPosition(0);
                            claw.servo1.setPosition(0);
                            claw.servo2.setPosition(0);
                            finiteState = FiniteState.DELIVERY_BUCKET;
                            clawTimer.reset();
                        }
                    }
                    break;
                }
                case DELIVERY_BUCKET: {
                    if (clawTimer.milliseconds() > 100){
                        if (currentGamepad1.left_bumper && ! previousGamepad1.left_bumper) {
                        claw.clawOpen();
                        finiteState = FiniteState.DELIVERY_END;
                        clawTimer.reset();
                        }
                    }
                    break;
                }
                case DELIVERY_END:{
                    if (clawTimer.milliseconds() > 100){
                        claw.servo1.setPosition(0);
                        claw.servo2.setPosition(0);
                        arm.extensionMotor.setTargetPosition(0);
                        arm.pivotMotor.setTargetPosition(0);
                        finiteState = FiniteState.INTAKE_GROUND_START;
                    }
                }
                case DELIVERY_SPECIMEN:{
                    if (currentGamepad2.a && !previousGamepad2.a){
                        arm.pivotMotor.setTargetPosition(0);
                        claw.clawOpen();
                        arm.extensionMotor.setTargetPosition(0);
                        claw.servo1.setPosition(0);
                        claw.servo2.setPosition(0);
                    }
                }
            }
        }
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        telemetry.addData("Loop Timer", loopTimer.milliseconds());
        telemetry.update();
    }
}
