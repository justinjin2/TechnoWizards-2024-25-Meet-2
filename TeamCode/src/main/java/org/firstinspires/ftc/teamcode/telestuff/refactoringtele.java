//package org.firstinspires.ftc.teamcode.telestuff;
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;
////import com.arcrobotics.ftclib.controller.PIDController;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
//import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;
//
//
//import org.firstinspires.ftc.teamcode.robot.robot;
////import org.firstinspires.ftc.teamcode.robot.RobotConstants;
//
//@TeleOp(name="telerefactor")
//public class refactoringtele extends LinearOpMode{
//    private Follower follower;
//
//    robot robot = new robot();
// //   RobotConstants constants = new RobotConstants();
//
//    public DcMotor leftFront;
//    public DcMotor rightFront;
//    public DcMotor leftRear;
//    public DcMotor rightRear;
//    public PIDFController liftPIDF;
//
//    public DcMotorEx armMotor;
//
//
//    public Servo wrist;
//    public Servo intakeservo;
//
//    enum intakeState { //INTAKE
//       START,
//        Ready,
//       Intake,
//        AFTERINTAKE
//    }
//
//    enum Deposit { //INTAKE
//        START,
//        Depositpos,
//       outtake
//    }
//
//    enum Speciman { //INTAKE
//        START,
//        Depositpos,
//        outtake,
//        endofouttake
//    }
//    double SpeedAdjust = 1;
//
//    HardwareMap hwMap = null;
//
//
//   public void initialize() {
//       follower = new Follower(hardwareMap);
//       leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
//       leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
//       rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
//       rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
//
//       armMotor = hwMap.get(DcMotorEx.class, "arm");
//       wrist = hwMap.get(Servo.class, "wrist");
//       intakeservo = hwMap.get(Servo.class, "intake");
//
//       leftFront.setDirection(DcMotor.Direction.FORWARD);
//       rightFront.setDirection(DcMotor.Direction.REVERSE);
//       leftRear.setDirection(DcMotor.Direction.FORWARD);
//       rightRear.setDirection(DcMotor.Direction.REVERSE);
//       armMotor.setDirection(DcMotor.Direction.FORWARD);
//
//       leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//       armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//       armMotor.setPower(0);
//
//       leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//       armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//       follower.startTeleopDrive();
//       armMotor.setTargetPosition(RobotConstants.STATIONARYPOS);
//       armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//       armMotor.setPower(0.9);
//       wrist.setPosition(RobotConstants.WRISTCENTERPOS);
//
//       intakeState intake = intakeState.START;
//       Deposit outtake = Deposit.START;
//       Speciman specdepo = Speciman.START;
//
//      // liftPIDF = new PIDFController(liftPIDFCoefficients);
//   }
//
//    @Override
//    public void runOpMode() {
//        work();
//    }
//
//    public void work() {
//        while (!isStarted() && !isStopRequested()) {
//        }
//
//        initialize();
//
//
////        frameTimer.resetTimer();
////        leftIntakeArm.getController().pwmEnable();
////        rightIntakeArm.getController().pwmEnable();
//
//        while (opModeIsActive()) {
//            driverControlUpdate();
//
//            telemetry();
//        }
//    }
//    public void telemetry() {
//        telemetry.addData("Desired Position", armMotor.getTargetPosition());
//        telemetry.addData("Current Position:",armMotor.getCurrentPosition());
//        telemetry.update();
//    }
//
//    public void driverControlUpdate() {
//      //  updateFrameTime();
//
//        drive();
//
//        buttonControls();
//
//      //  teleopLiftControlUpdate();
//       // updateServoMechanisms();
//       // fineAdjustControls();
//    }
//
//    public void autonomousControlUpdate() {
//       // updateFrameTime();
//
//      //  updateLift();
//
//      //  updateServoMechanisms();
//    }
//
   // public void drive (){
      //  follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        //follower.update();
//
//    }
//    public void buttonControls() {
//        if (gamepad1.y) {
//            wrist.setPosition(RobotConstants.WRISTCENTERPOS);
//          //  armMotor.setTargetPosition(PREINTAKEARMPOS);
//            intakeservo.setPosition(RobotConstants.INTAKEOFF);
//           // intake = intakeState.START;
//        }
//
//        if (gamepad2.a) {
//            wrist.setPosition(RobotConstants.WRISTCENTERPOS);
//           // robot.armMotor.setTargetPosition(DRIVEARMPOS);
//            robot.intake.setPosition(RobotConstants.INTAKEOFF);
//        }
//        if (gamepad1.left_bumper) {
//            SpeedAdjust = 4;
//        } else if (gamepad1.right_bumper) {
//            SpeedAdjust = 2;
//        }
//        if (gamepad2.b) {
//            wrist.setPosition(RobotConstants.WRISTSPECPOS);
//          //  armMotor.setTargetPosition(SPECARMPOS);
//        }
//        if (gamepad2.left_bumper) {
//            robot.armMotor.setTargetPosition(2550);
//        }
//        if (gamepad2.right_bumper) {
//            armMotor.setTargetPosition(50);
//        }
//        if (gamepad1.x) {//left
//            wrist.setPosition(0.99);
//        }
//        if (gamepad1.b) {//right
//            wrist.setPosition(0.01);
//        }
//        armMotor.setPower(1);
//
//    }
////        switch (intake) { // scoring pos with lift
////        case START:
////            if (gamepad2.dpad_down) {
////                armMotor.setTargetPosition(PREINTAKEARMPOS);
////                wrist.setPosition(RobotConstants.WRISTCENTERPOS);
////
////                intake = intakeState.Intake; //OUTTAKE POSITIONS, DIF HEIGHTS (black frame thing for pixels)
////            }
////            break;
////        case Intake:
////            if (gamepad1.a) {
////                wrist.setPosition(RobotConstants.WRISTCENTERPOS);
////                armMotor.setTargetPosition(INTAKEARMPOS);
////                intakeservo.setPosition(RobotConstants.INTAKEON);
////                intake = intakeState.START;
////            }
////            break;
////
////
////
////    }
////
////        switch (outtake) { // scoring pos with lift
////        case START:
////
////            if (gamepad2.dpad_up) {
////                wrist.setPosition(RobotConstants.WRISTCENTERPOS);
////                armMotor.setTargetPosition(DEPOSITARMPOS);
////                /// robot.intake.setPosition(0.5);
////                outtake = Deposit.outtake;
////            }
////            break;
////        case outtake:
////            if (gamepad1.right_trigger >=0.5) {
////                intakeservo.setPosition(RobotConstants.INTAKEOUT);
////                outtake = Deposit.START;
////            }
////            break;
////    }
////        switch (specdepo) {
////        case START:
////
////            if (gamepad1.dpad_down) {
////                armMotor.setTargetPosition(AFTERSPECDROPPOS);
////                specdepo = Speciman.START;
////            }
////            break;
////    }
//
//
//}
//
//
//
////}
