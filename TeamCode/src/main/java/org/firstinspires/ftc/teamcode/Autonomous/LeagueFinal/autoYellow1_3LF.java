package org.firstinspires.ftc.teamcode.Autonomous.LeagueFinal;

import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.AUTO_INTAKE_END;
import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.CLIP_DELIVERY_READY;
import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.DELIVERY_SPECIMEN;
import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.PIVOT_READY;
import static org.firstinspires.ftc.teamcode.TeleOp.meet2.FiniteState.SCORE_PRELOAD;

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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

;


//@Disabled
@Autonomous(name = "leftAuto1+3LF", group = "Auto" )
public class autoYellow1_3LF extends OpMode {
    private Follower follower;
    Arm arm = new Arm();
    Claw claw = new Claw();
    private int clawServoTime = 400;
    private Timer pathTimer, opModeTimer, loopTimer;

    ElapsedTime waitingTimer;

    ElapsedTime clawTimer;
    private Pose currentPose;
    private FiniteState finiteState = SCORE_PRELOAD;
    private int numberOfDelivery = 0;

    // private final Pose startPose = new Pose (135, 89, Math.toRadians(0));
    /*
    0 heading with left edge parallel to the 2nd tile on the left.
     */
    private final Pose startPose = new Pose(133, 55, Math.toRadians(0));
    private Pose scorePreloadPose = new Pose(111.2, 62, Math.toRadians(0));
    private Pose scoreBucketPose = new Pose(131.25, 23.6, Math.toRadians(135));
    private Pose scoreBucketPose2 = new Pose(131.25, 23.5, Math.toRadians(135));
    private Pose scoreBucketPose3 = new Pose(131.25, 23.5, Math.toRadians(135));

    private Pose groundIntakePose1 = new Pose(113.75, 36, Math.toRadians(181));
    private Pose groundIntakePose2 = new Pose(117, 23.75, Math.toRadians(180));
    private Pose groundIntakePose3 = new Pose(111, 21.75, Math.toRadians(220));


//private Pose wallIntakeAdjustPose = new Pose(130.5, 116, Math.toRadians(0));

    private Pose parkPose = new Pose(77, 57.8, Math.toRadians(270));
    private Point parkPoseControl = new Point(96,20);

    private int pathState;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, groundToScore1, groundToScore2, groundToScore3, parkPath;
    private Path groundIntake1, groundIntake2, groundIntake3;

    /**
     * Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.
     **/
    public void buildPaths() {

        //scorePreload = new Path(new BezierCurve(new Point(startPose), scoreControlPoint, new Point(scorePose)));
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePreloadPose)));

        groundIntake1 = new Path(new BezierLine(new Point(scorePreloadPose), new Point(groundIntakePose1)));
        groundToScore1 = new Path(new BezierLine(new Point(groundIntakePose1), new Point(scoreBucketPose)));

        groundIntake2 = new Path(new BezierLine(new Point(scoreBucketPose), new Point(groundIntakePose2)));
        groundToScore2 = new Path(new BezierLine(new Point(groundIntakePose2), new Point(scoreBucketPose2)));

        groundIntake3 = new Path(new BezierLine(new Point(scoreBucketPose2), new Point(groundIntakePose3)));
        groundToScore3 = new Path(new BezierLine(new Point(groundIntakePose3), new Point(scoreBucketPose3)));

        parkPath = new Path(new BezierCurve(new Point(scoreBucketPose3), parkPoseControl, new Point(parkPose)));
       // parkPath = new Path(new BezierCurve(new Point(scoreBucketPose), parkPoseControl, new Point(parkPose)));
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

    }


    public void autonomousPathUpdate() {
        loopTimer.resetTimer();
        switch (finiteState) {
            case SCORE_PRELOAD: //First Scored Path
                follower.followPath(scorePreload, false);
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePreloadPose.getHeading());
                finiteState = PIVOT_READY;
                pathTimer.resetTimer();
                break;

            case PIVOT_READY:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    arm.movePivotMotor(arm.maximumPivot, arm.motorPower);
                    arm.moveExtensionMotor(arm.specimenDeliverExtension, arm.motorPower);
                    finiteState = CLIP_DELIVERY_READY;
                }
                break;
            /*****************************
             * Move the wrist down;
             */
            case CLIP_DELIVERY_READY:
                if (!follower.isBusy() && (arm.pivotMotor.getCurrentPosition() + 15) > arm.maximumPivot) {
                    claw.wristDeliverSpecimen();
                    clawTimer.reset();
                    arm.movePivotMotor(arm.maximumPivot-25, arm.motorPower);
                    finiteState = DELIVERY_SPECIMEN;
                }
                break;
            /***************************
             * Wait for claw go down so the claw can open to release the specimen
             ******************/
            case DELIVERY_SPECIMEN:
                if (clawTimer.milliseconds() > clawServoTime+200) {
                    claw.clawOpen();
                    clawTimer.reset();
                    finiteState = FiniteState.INTAKE_GROUND_START;
                    numberOfDelivery = numberOfDelivery + 1;
                }
                break;

            case INTAKE_GROUND_START:
                if (clawTimer.milliseconds() > clawServoTime) {
                    if (numberOfDelivery == 1) {
                        follower.followPath(groundIntake1, false);
                        groundIntake1.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), groundIntakePose1.getHeading());
                    }
                    if (numberOfDelivery == 2) {
                        follower.followPath(groundIntake2, false);
                        groundIntake2.setLinearHeadingInterpolation(scoreBucketPose.getHeading(), groundIntakePose1.getHeading());
                    }
                    if (numberOfDelivery == 3) {
                        follower.followPath(groundIntake3, false);
                        groundIntake3.setLinearHeadingInterpolation(scoreBucketPose.getHeading(), groundIntakePose3.getHeading());
                        //claw.wristAutoRight();
                    }
                    arm.movePivotMotor(arm.groundIntakePivotReady + 25, arm.motorPower);
                    arm.moveExtensionMotor(arm.groundIntakeExtension, arm.motorPower);

                    claw.clawOpen();
                    claw.wristDown();

                    clawTimer.reset();
                    finiteState = FiniteState.DIFFY_WRIST;//INTAKE_GROUND_PIVOT;

                }
                break;


            case DIFFY_WRIST:
                if (numberOfDelivery == 3){
                    claw.wristAutoRight();
                    finiteState = FiniteState.INTAKE_GROUND_PIVOT;
                }
                else {
                    finiteState = FiniteState.INTAKE_GROUND_PIVOT;
                }
                break;

            case INTAKE_GROUND_PIVOT:
                if ((arm.pivotMotor.getCurrentPosition() - 25 <= arm.groundIntakePivotReady) && (clawTimer.milliseconds() > clawServoTime) && (!follower.isBusy())) {
                    arm.movePivotMotor(arm.groundIntakePivot, arm.motorPower);
                    clawTimer.reset();
                    finiteState = FiniteState.INTAKE_GROUND_CLAW;
                }
                break;

            case INTAKE_GROUND_CLAW:
                if (clawTimer.milliseconds() > 150) {
                    claw.clawClose();
                    clawTimer.reset();
                    finiteState = FiniteState.INTAKE_GROUND_END;
                }
                break;

            case INTAKE_GROUND_END:
                if (clawTimer.milliseconds() > clawServoTime) /*|| (arm.getSpecimenColorSensor() <= 30)*/ {
                    if (numberOfDelivery == 1) {
                        follower.followPath(groundToScore1, false);
                        groundToScore1.setLinearHeadingInterpolation(groundIntakePose1.getHeading(), scoreBucketPose.getHeading());
                        arm.movePivotMotor(arm.maximumPivotBucket, arm.motorPower);
                        claw.wristDeliver();
                        clawTimer.reset();
                        finiteState = FiniteState.DELIVERY_HIGH_BUCKET_PIVOT;
                    }
                    if (numberOfDelivery == 2) {
                        follower.followPath(groundToScore2, false);
                        groundToScore2.setLinearHeadingInterpolation(groundIntakePose2.getHeading(), scoreBucketPose.getHeading());
                        arm.movePivotMotor(arm.maximumPivotBucket, arm.motorPower);
                        claw.wristDeliver();
                        clawTimer.reset();
                        finiteState = FiniteState.DELIVERY_HIGH_BUCKET_PIVOT;
                    }
                    if (numberOfDelivery == 3) {
                        follower.followPath(groundToScore3, false);
                        groundToScore3.setLinearHeadingInterpolation(groundIntakePose3.getHeading(), scoreBucketPose.getHeading());
                        arm.movePivotMotor(arm.maximumPivotBucket, arm.motorPower);
                        claw.wristDeliver();
                        clawTimer.reset();
                        finiteState = FiniteState.DELIVERY_HIGH_BUCKET_PIVOT;
                    }
                    if (numberOfDelivery == 4) {
                       finiteState = AUTO_INTAKE_END;
                    }
                }
                break;

            case DELIVERY_HIGH_BUCKET_PIVOT:
                if ((arm.pivotMotor.getCurrentPosition() + 400 >= arm.maximumPivotBucket) && (!follower.isBusy())) {
                    arm.moveExtensionMotor(arm.maximumDeliveryExtension, arm.motorPower);
//                    if(numberOfDelivery == 3){
//                        claw.wristAutoReset();
//                    }
                    finiteState = FiniteState.DELIVERY_HIGH_BUCKET;
                }
                break;

            case DELIVERY_HIGH_BUCKET:
                    /***********
                     * pay attention: change the if condition to be the condition above
                     * if extension motor works
                     */
                    if ((arm.extensionMotor.getCurrentPosition() + 100 >= arm.maximumDeliveryExtension) && (!follower.isBusy())) {
                        claw.clawOpen();
                        clawTimer.reset();
                        numberOfDelivery = numberOfDelivery + 1;
                        if (numberOfDelivery <= 3) {
                            finiteState = FiniteState.DELIVER_WAIT;//INTAKE_GROUND_START;
                        }
                        if (numberOfDelivery == 4) {
                            finiteState = FiniteState.INTAKE_GROUND_END;
                        }
                    }
                break;

            case DELIVER_WAIT:
                if (clawTimer.milliseconds() > clawServoTime){
                    claw.wristDown();
                    clawTimer.reset();
                    finiteState = FiniteState.EXTENSION_RESET_BUCKET;
                }
                break;

            case EXTENSION_RESET_BUCKET:
                if (clawTimer.milliseconds() > clawServoTime) {
                    arm.moveExtensionMotor(arm.minimumExtension, arm.motorPower);
                    finiteState = FiniteState.PIVOT_RESET_BUCKET;
                }
                break;

            case PIVOT_RESET_BUCKET:
                if (arm.extensionMotor.getCurrentPosition() <= arm.minimumExtension + 500) {
                    arm.movePivotMotor(arm.groundIntakeEndPivot, arm.motorPower);
                    clawTimer.reset();
                    finiteState = FiniteState.INTAKE_GROUND_START;
                }
                break;


            case AUTO_INTAKE_END:
                if (clawTimer.milliseconds() > clawServoTime) {
                    follower.followPath(parkPath);
                    parkPath.setLinearHeadingInterpolation(scoreBucketPose.getHeading(), parkPose.getHeading());
                    finiteState = FiniteState.END_AUTO;
                }
                break;

            case PARK_SERVO:
                if(!follower.isBusy() || opModeTimer.getElapsedTimeSeconds()>28) {
                    arm.parkServo.setPosition(arm.parkServoUp);
                    follower.breakFollowing();
                    finiteState = FiniteState.IDLE;
                }

        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /*********
     public void setAutoState(){
     pathTimer.resetTimer();
     }
     ****************/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        switch (finiteState) {
            case END_AUTO:
                arm.resetTouch();
                finiteState = FiniteState.PARK_SERVO;
                break;
//          telemetry.addData("path state", pathState);
//          telemetry.addData("x", follower.getPose().getX());
//          telemetry.addData("y", follower.getPose().getY());
//          telemetry.addData("percentage of complete path:", (follower.getCurrentTValue() * 100));
        }
    }

    @Override
    public void init() {
        loopTimer = new Timer();
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
        arm.parkServo.setPosition(arm.parkServoDown);
    }

    @Override
    public void init_loop () {
// Camera code
    }

    @Override
    public void start() {
        buildPaths();
        opModeTimer.resetTimer();
        setPathState(0);
    }

}

