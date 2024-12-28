package org.firstinspires.ftc.teamcode.Testing;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * This OpMode executes a POV Game style Teleop for a direct drive robot
 * The code is structured as a LinearOpMode
 *
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the arm using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="azClawTest", group="TEst")
@Disabled
public class azClawTest extends LinearOpMode {

    /* Declare OpMode members. */

    public Servo claw = null;
    public Servo leftDIF = null;
    public Servo rightDIF = null;
    public Servo axonTest = null;
    double clawOffset = 0;
    double rightDIFOffset = 0;
    double leftDIFOffset = 0;
    double axonTestOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;

        // Define and Initialize Motors
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        claw   = hardwareMap.get(Servo.class, "claw");
        rightDIF = hardwareMap.get(Servo.class, "rightDIF");
        leftDIF = hardwareMap.get(Servo.class, "leftDIF");
        axonTest = hardwareMap.get(Servo.class, "axon");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
        telemetry.update();
        rightDIFOffset = MID_SERVO;
        leftDIFOffset = MID_SERVO;
        clawOffset = MID_SERVO;

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forward, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            // drive = -gamepad1.left_stick_y;
            //turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            // left  = drive + turn;
            //right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
//            max = Math.max(Math.abs(left), Math.abs(right));
//            if (max > 1.0)
//            {
//                left /= max;
//                right /= max;
//            }

            // Output the safe vales to the motor drives.
//            leftDrive.setPower(left);
//            rightDrive.setPower(right);
//
            // Use gamepad left & right Bumpers to open and close the claw


            //else if (gamepad1.b){
            //   arm.setPower(0.0);
            //}
//            if (touchSensor.isPressed()){
//                int i;
//                for(i=0; i<100; i++){
//
            currentGamepad1.copy(gamepad1); //                leftArm.setPower(0.0);
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
                clawOffset += CLAW_SPEED;
            }
            else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                clawOffset -= CLAW_SPEED;
            }
            claw.setPosition(MID_SERVO + clawOffset);


            if (currentGamepad1.y && !previousGamepad1.y) {
                rightDIFOffset -= CLAW_SPEED;
                leftDIFOffset += CLAW_SPEED;
            }
            else if (currentGamepad1.a && !previousGamepad1.a) {
                rightDIFOffset += CLAW_SPEED;
                leftDIFOffset -= CLAW_SPEED;
            }
            else if (currentGamepad1.x && !previousGamepad1.x) {
                rightDIFOffset -= CLAW_SPEED;
                leftDIFOffset -= CLAW_SPEED;
            }
            else if (currentGamepad1.b && !previousGamepad1.b) {
                rightDIFOffset += CLAW_SPEED;
                leftDIFOffset += CLAW_SPEED;
            }
            rightDIF.setPosition(-MID_SERVO + rightDIFOffset);
            leftDIF.setPosition(MID_SERVO + leftDIFOffset);


            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                axonTestOffset += CLAW_SPEED;
            } else if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                axonTestOffset -= CLAW_SPEED;
            }
            axonTest.setPosition(MID_SERVO + axonTestOffset);
            previousGamepad1.copy(currentGamepad1);
            // Move both servos to new position.  Assume servos are mirror image of each other.
//            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
//            leftClaw.setPosition(MID_SERVO + clawOffset);
//            rightClaw.setPosition(MID_SERVO - clawOffset);

            // Use gamepad buttons to move arm up (Y) and down (A)
//            if (gamepad1.y)
//                leftArm.setPower(ARM_UP_POWER);
//            else if (gamepad1.a)
//                leftArm.setPower(ARM_DOWN_POWER);
//            else
//                leftArm.setPower(0.0);

//            if (gamepad1.a) {
//                leftArm.setPower(ARM_DOWN_POWER);
//            } else {
//                leftArm.setPower(0.0);
//            }
//            // Send telemetry message to signify robot running;
            telemetry.addData("Claw position",  "Offset = %.2f", clawOffset);
            telemetry.addData("Right DIF position",  "Offset = %.2f", rightDIFOffset);
            telemetry.addData("Left DIF position",  "Offset = %.2f", leftDIFOffset);
            telemetry.addData("Axon test position",  "Offset = %.2f", axonTestOffset);

////            telemetry.addData("left",  "%.2f", left);
//            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            sleep(50);
        }
    }
}

