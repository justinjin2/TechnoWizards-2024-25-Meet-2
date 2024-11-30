package org.firstinspires.ftc.teamcode.TeleOp.meet2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;
import org.firstinspires.ftc.teamcode.Hardware.meet2.DriveTrain;
import org.firstinspires.ftc.teamcode.TeleOp.meet1.meet1tele;

/**
 * This is the TeleOpEnhancements OpMode. It is an example usage of the TeleOp enhancements that
 * Pedro Pathing is capable of.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/21/2024
 */
@TeleOp(name = "Meet2TeleOp", group = "Test")
public class Meet2TeleOp extends LinearOpMode {
    Arm arm = new Arm();
    Claw claw = new Claw();
    DriveTrain driveTrain = new DriveTrain();
    double SpeedAdjust = 1;
    public Telemetry telemetryA;
    private FiniteState finiteState = FiniteState.IDLE;
    //telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());


    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        arm.init(hardwareMap);
        claw.init(hardwareMap);
        driveTrain.init(hardwareMap);

        /**
         * This runs the OpMode. This is only drive control with Pedro Pathing live centripetal force
         * correction.
         */
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // YOUR WORK
            driveTrain.rightRear.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.leftRear.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.rightFront.setPower((-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) / SpeedAdjust);
            driveTrain.leftFront.setPower((-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) / SpeedAdjust);

            telemetry();

            switch (finiteState) {
                case INTAKE_GROUND_START:{
                    if (gamepad1.a) {
                        arm.pivotMotor.setTargetPosition(0);
                        arm.extensionMotor.setTargetPosition(0);
                        claw.servo1.setPosition(0);
                        claw.servo2.setPosition(0);
                        claw.claw.setPosition(claw.OPEN_SERVO);
                        finiteState = finiteState.INTAKE_GROUND; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                }
                case INTAKE_GROUND:{
                    if (gamepad1.right_bumper){
                        claw.claw.setPosition(claw.CLOSE_SERVO);
                        claw.servo1.setPosition(0);
                        claw.servo2.setPosition(0);
                        arm.extensionMotor.setTargetPosition(0);
                        arm.pivotMotor.setTargetPosition(0);
                        finiteState = finiteState.INTAKE_GROUND_END; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
                    }
                    break;
                }
                case INTAKE_GROUND_END:{
                    //set delivery into ready state
                }
            }
        }
    }
        public void telemetry() {
            telemetryA.addData("Robot State", "initialization");
//            telemetryA.addData("Outtake State", outtakeState);
//            telemetryA.addData("Transfer State", transferState);
//            telemetryA.addData("Lift Position", liftEncoder.getCurrentPosition());
//            telemetryA.addData("Lift Target Position", liftTargetPosition);
//            telemetryA.addData("Lift Current", liftEncoder.getCurrent(CurrentUnit.MILLIAMPS));
//            telemetryA.addData("Outtake Wrist Offset", outtakeWristOffset);
//            telemetryA.addData("Intake Arm Position", leftIntakeArm.getPosition());
//            telemetryA.addData("Intake Arm Target Position", intakeArmTargetPosition);
//            telemetryA.addData("Intake Arm Target State", intakeArmTargetState);
//            telemetryA.addData("Intake Arm PWM Status", leftIntakeArm.getController().getPwmStatus());
//            telemetryA.addData("Intake Arm Connection Info", leftIntakeArm.getConnectionInfo());
            telemetryA.update();
        }
}
