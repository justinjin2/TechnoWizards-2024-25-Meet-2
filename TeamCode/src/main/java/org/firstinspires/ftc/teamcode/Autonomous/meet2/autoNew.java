package org.firstinspires.ftc.teamcode.Autonomous.meet2;

import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.END_AUTO;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;
import org.firstinspires.ftc.teamcode.Hardware.meet2.Claw;
import org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

//@Disabled
@Autonomous (name = "Auto New", group = "test" )
public class autoNew extends OpMode {
    private Follower follower;
    Arm arm = new Arm();
    Claw claw = new Claw();

    private Timer pathTimer, opModeTimer;

    ElapsedTime waitingTimer;

    ElapsedTime clawTimer;
    private Pose currentPos;
    private FiniteState finiteState = FiniteState.SCORE_PRELOAD;
    private int numberOfDelivery=0;

    private final Pose startPose = new Pose (137, 102, Math.toRadians(270));
    // private Point scoreControlPoint= new Point(120, 96);
    private Pose scorePose = new Pose (113, 76, Math.toRadians(0));
    private Pose scorePose1 = new Pose (113, 72, Math.toRadians(0));
    private Pose scorePose2 = new Pose (113, 70, Math.toRadians(0));
    private Pose scorePose3 = new Pose (113, 67, Math.toRadians(0));

    private Pose wallIntakePose = new Pose(130, 116, Math.toRadians(0));

//private Pose wallIntakeAdjustPose = new Pose(130.5, 116, Math.toRadians(0));

    private Pose pushPickup1ReadyPose = new Pose(85,114, Math.toRadians(0));
    private Pose pickup1ControlPose1 = new Pose (122, 107, Math.toRadians(0));
    private Pose pickup1ControlPose2 = new Pose (100, 105, Math.toRadians(0));
    private Pose endPushPose1 =new Pose(120,pushPickup1ReadyPose.getY(), Math.toRadians(0));

    private Pose pushPickup2ReadyPose = new Pose(83,124, Math.toRadians(0));
    private Pose endPushPose2 =new Pose(120,pushPickup2ReadyPose.getY(), Math.toRadians(0));
    private Pose pickup2ControlPose = new Pose(112,102, Math.toRadians(0));

    private Pose pushPickup3ReadyPose = new Pose(83,130, Math.toRadians(0));
    private Pose endPushPose3 =new Pose(120,pushPickup3ReadyPose.getY(), Math.toRadians(0));
    private Pose pickup3ControlPose = new Pose(112,112, Math.toRadians(0));

    private Pose parkPose = new Pose (123, 123, Math.toRadians(0));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, wallIntake, wallToScore1,wallToScore2,wallToScore3;
    private Path wallIntake2, wallIntake3, endPickToWallPath;
    private PathChain  pushPickup1, pushPickup2, pushPickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        //scorePreload = new Path(new BezierCurve(new Point(startPose), scoreControlPoint, new Point(scorePose)));
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        wallIntake = new Path(new BezierLine(new Point(scorePose), new Point(wallIntakePose)));

        wallToScore1 = new Path(new BezierLine(new Point(wallIntakePose), new Point(scorePose1)));
        wallIntake2= new Path(new BezierLine(new Point(scorePose1), new Point(wallIntakePose)));

        wallToScore2 = new Path(new BezierLine(new Point(wallIntakePose), new Point(scorePose2)));
        wallIntake3= new Path(new BezierLine(new Point(scorePose2), new Point(wallIntakePose)));

        wallToScore3 = new Path(new BezierLine(new Point(wallIntakePose), new Point(scorePose3)));

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our pushPickup1 PathChain. We are using  a BezierCurve with 4 points, which is a curved line that is curved based off of the control point */
        pushPickup1= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup1ReadyPose), new Point(endPushPose1)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose1), new Point(endPushPose1.getX()-5, endPushPose1.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose1.getX()-5, endPushPose1.getY()), new Point(wallIntakePose)))
                .setConstantHeadingInterpolation(0)
                .build();

        pushPickup2= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup1ReadyPose), new Point(endPushPose1)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(endPushPose1), new Point(pickup2ControlPose), new Point(pushPickup2ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup2ReadyPose), new Point(endPushPose2)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose2), new Point(endPushPose2.getX()-5, endPushPose2.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose2.getX()-5, endPushPose2.getY()), new Point(wallIntakePose)))
                .setConstantHeadingInterpolation(0)
                .build();

        endPickToWallPath = new Path(new BezierLine(new Point(endPushPose2.getX()-5, endPushPose2.getY()),new Point(wallIntakePose)));

        pushPickup3= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose1), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup1ReadyPose), new Point(endPushPose1)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(endPushPose1), new Point(pickup2ControlPose), new Point(pushPickup2ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup2ReadyPose), new Point(endPushPose2)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierCurve(new Point(endPushPose2), new Point(pickup3ControlPose), new Point(pushPickup3ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup3ReadyPose), new Point(endPushPose3)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose3), new Point(endPushPose3.getX()-5, endPushPose3.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose3.getX()-5, endPushPose3.getY()), new Point(wallIntakePose)))
                .setConstantHeadingInterpolation(0)
                .build();

    }

    public void autonomousPathUpdate() {
        switch  (finiteState) {
            /*********************
             * clip the preload sample
             * *************************************
             * Initial: claw wristdown & close
             * pivot arm 0;
             * extBoxTube power on;
             * robot run to score position
             */
            case SCORE_PRELOAD: //First Scored Path
                telemetry.addData("run to", "preload score Pose");
                telemetry.update();
                follower.followPath(scorePreload, false);
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
                finiteState = FiniteState.PIVOT_READY;
                pathTimer.resetTimer();
                break;
            /*****************************
             * Move the arm and extension,
             */
            case PIVOT_READY:
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    arm.movePivotMotor(arm.maximumPivot, arm.motorPower);
                    arm.moveExtensionMotor(arm.specimenDeliverExtension, arm.motorPower);
                    finiteState = FiniteState.CLIP_DELIVERY_READY;
                }
                break;
            /*****************************
             * Move the wrist down;
             */
            case CLIP_DELIVERY_READY:
                if (!follower.isBusy() && (arm.pivotMotor.getCurrentPosition() + 15) > arm.maximumPivot) {
                    claw.wristDeliverSpecimen();
                    finiteState = FiniteState.DELIVERY_SPECIMEN;
                    clawTimer.reset();
                }
                break;

            /***************************
             * Wait for claw go down so the claw can open to release the specimen
             ******************/
            case DELIVERY_SPECIMEN:
                if (clawTimer.milliseconds() > 400) {
                    claw.clawOpen();
                    clawTimer.reset();
                    finiteState = FiniteState.PIVOT_RESET_SPECIMEN;
                    numberOfDelivery = numberOfDelivery + 1;
                }
                break;

            /************
             * wait for claw to Open  so it's save to move the arm back to wall intake;
             *   Move arm to wall intake position meanwhile, prepare to adjust the extension
             */

            case PIVOT_RESET_SPECIMEN:
                if (clawTimer.milliseconds() > 200) {
                    arm.movePivotMotor(arm.wallIntakePivot, arm.motorPower);
                    claw.wristCenter();
                    clawTimer.reset();
                    finiteState = FiniteState.EXTENSION_RESET_SPECIMEN;
                }
                break;
            /********
             * Adjust the extension arm for the string while the pivot arm is moving down
             * but not to the wall intake position
             */
            case EXTENSION_RESET_SPECIMEN:
                if (arm.pivotMotor.getCurrentPosition() <= arm.resetPivot) {
                    arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                    finiteState = FiniteState.DECISION; //INTAKE_WALL_START;
                }
                break;

            /************
             * Decide if the robot needs to push samples or go to wall intake
             */
            case DECISION:
                if (numberOfDelivery == 2)
                    finiteState = FiniteState.SAMPLE_PUSH;
                else
                    finiteState = FiniteState.INTAKE_WALL_START;
                break;

            case SAMPLE_PUSH:
                follower.followPath(pushPickup3, false);
                pathTimer.resetTimer();
                finiteState = FiniteState.INTAKE_WALL_PRE_END;
                break;

            case SAMPLE_PUSH_DONE:
                if (!follower.isBusy()) {
                    follower.followPath(endPickToWallPath, false);
                    endPickToWallPath.setConstantHeadingInterpolation(0);
                    pathTimer.resetTimer();
                    finiteState = FiniteState.INTAKE_WALL_PRE_END;
                }
                break;

            case INTAKE_WALL_START:
                follower.followPath(wallIntake, false);
                wallIntake.setConstantHeadingInterpolation(0);
                claw.wristCenter();
                pathTimer.resetTimer();
                finiteState = FiniteState.INTAKE_WALL_PRE_END;
                break;

            case INTAKE_WALL_PRE_END:
                if (!follower.isBusy() && (arm.pivotMotor.getCurrentPosition() + 50) > arm.wallIntakePivot) {
                    claw.clawClose();
                    clawTimer.reset();
                    finiteState = FiniteState.INTAKE_WALL_END;
                }
                break;

            case INTAKE_WALL_END:
                if (clawTimer.milliseconds() > 200) {
                    claw.wristReadySpecimen();

                    if (numberOfDelivery == 1) {
                        follower.followPath(wallToScore1, false);
                        wallToScore1.setConstantHeadingInterpolation(0);
                        pathTimer.resetTimer();
                        finiteState = FiniteState.PIVOT_READY;
                    }

                    if (numberOfDelivery == 2) {
                        follower.followPath(wallToScore2, false);
                        wallToScore2.setConstantHeadingInterpolation(0);
                        pathTimer.resetTimer();
                        finiteState = FiniteState.PIVOT_READY;
                    }

                    if (numberOfDelivery == 3) {
                        finiteState= FiniteState.END_AUTO;
                    }
                }
                break;
//            case END_AUTO:
//                if(numberOfDelivery==3){
//                    arm.resetTouch();
//                    numberOfDelivery=10;
//                    finiteState=FiniteState.IDLE;
//                }
        }
        telemetry.addData("finiteState", finiteState);
        telemetry.update();

    }


    @Override
    public void loop () {
        follower.update();
        autonomousPathUpdate();
        switch(finiteState){
            case END_AUTO:
                arm.resetTouch();
                finiteState=FiniteState.IDLE;
                break;
        }
        telemetry.addData("finite state", finiteState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("number of Deliversy", numberOfDelivery);
        telemetry.addData("percentage of complete path:", (follower.getCurrentTValue() * 100));
        telemetry.update();

    }

    @Override
    public void init () {


        pathTimer = new Timer();
        opModeTimer = new Timer();

        clawTimer = new ElapsedTime();
        arm.intakeTimer = new ElapsedTime();

        opModeTimer.resetTimer();
        pathTimer.resetTimer();
        clawTimer.reset();


        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        arm.init(hardwareMap);
        claw.init(hardwareMap);

        claw.wristUp();
        claw.clawClose();
        arm.initRunExtMotor(arm.motorPower);
        finiteState = FiniteState.SCORE_PRELOAD;
    }

    @Override
    public void init_loop () {
// Camera code
    }

    @Override
    public void start () {
        buildPaths();
        opModeTimer.resetTimer();
    }
}
