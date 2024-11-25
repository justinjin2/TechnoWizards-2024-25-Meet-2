package org.firstinspires.ftc.teamcode.autotesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.robot.robot;




@Autonomous (name = "bad", group = "test" )
public class bad extends OpMode {
    private Timer pathTimer, opModeTimer, scanTimer;
    robot robot = new robot();
    ElapsedTime waitTimer1 = new ElapsedTime();

    //Spike Mark Poses


//Backdrop Poses

//White stack Poses
int DEPOSITARMPOS = 2970;
    int PREINTAKEARMPOS = 4225;
    int DRIVEARMPOS = 1000;
    int INTAKEARMPOS = 4735;
    int SPECARMPOS = 2900;
    int AFTERSPECDROPPOS = 3200;
    double WRISTCENTERPOS = 0.512;
    double INTAKEIN = 0;
    double INTAKEOUT = 1;
    double INTAKEOFF = 0.5;

    private Pose preloadspecimanpose, samplerightpose, samplemiddlepose, sampleleftpose, scoresampleinbasketpose;

    private Pose startPose = new Pose(137, 41, Math.toRadians(270));

    private Follower follower;

    private Path specimanPreloaddeposit, goToSampleright, goToSamplemiddle, goToSampleleft, depositcycle1, depositcycle2, depositcycle3;

    private int pathState;
    private int actionState, clawState;


    public void setBackdropGoalPose(){
//


        preloadspecimanpose = new Pose(110, 62, Math.toRadians(270));

//        samplerightpose= new Pose(105.404, 24.098, Math.toRadians(200));
//        samplemiddlepose =  new Pose(138,120, Math.toRadians(180));
//        sampleleftpose = new Pose(138,120, Math.toRadians(200));
//        scoresampleinbasketpose  = new Pose(127.739,13.322, Math.toRadians(145));

    }
    public void buildPaths() {
        Point tosampleleftmidpoint;
        tosampleleftmidpoint = new Point(113.829, 39.576, Point.CARTESIAN);

        specimanPreloaddeposit = new Path(new BezierLine(new Point(startPose), new Point(preloadspecimanpose)));
//        goToSampleright = new Path(new BezierCurve(new Point(preloadspecimanpose), tosampleleftmidpoint, new Point(samplerightpose)));
//        depositcycle1 = new Path(new BezierLine(new Point(samplerightpose), new Point(scoresampleinbasketpose)));
//        goToSamplemiddle = new Path(new BezierLine(new Point(scoresampleinbasketpose), new Point(samplemiddlepose)));
//        depositcycle2 = new Path(new BezierLine(new Point(samplemiddlepose), new Point(scoresampleinbasketpose)));
//        goToSampleright = new Path(new BezierLine(new Point(scoresampleinbasketpose), new Point(sampleleftpose)));
//        depositcycle3 = new Path(new BezierLine(new Point(sampleleftpose), new Point(scoresampleinbasketpose)));

//firstcyclebackdrop = new Path(new BezierCurve(new Point(firstCycleStackPose), new Point(1,2, Point.CARTESIAN)))
    }

//cycleToBackboard = follower.pathBuilder()

    //.build();
    public void autonomousPathUpdate(){
        switch(pathState){
            case 10: //First Scored Path
//                robot.armMotor.setPower(1);
//                robot.armMotor.setTargetPosition(SPECARMPOS);
//                robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                robot.wrist.setPosition(1);

                follower.followPath(specimanPreloaddeposit);
                //When starting to move instantly moves to case 11
                setPathState(11);
                break;
            case 11: //While moving check how far away from wall and start interpolating

                specimanPreloaddeposit.setLinearHeadingInterpolation(startPose.getHeading(),preloadspecimanpose.getHeading());

                setPathState(24);
                break;
//            case 12:
//                if(pathTimer.getElapsedTimeSeconds() > 1){
//                    follower.followPath(goToSampleright, false);
//                    setPathState(13);
//
//                }
//                break;
//            case 13:
//                goToSampleright.setLinearHeadingInterpolation(preloadspecimanpose.getHeading(), samplerightpose.getHeading());
//
//                setPathState(14);
//                break;
//            case 14:
//                if(pathTimer.getElapsedTimeSeconds() > 1.9){
//                    follower.followPath(depositcycle1, false);
//                    setPathState(15);
//
//                }
//                break;
//
//            case 15:
//
//                depositcycle1.setLinearHeadingInterpolation(samplerightpose.getHeading(), scoresampleinbasketpose.getHeading());
//                setPathState(16);
//                break;
//            case 16:
//                if(pathTimer.getElapsedTimeSeconds() > 2.7) {
//                    follower.followPath(goToSamplemiddle, false);
//                    setPathState(17);
//                }
//                break;
//            case 17:
//
//                goToSamplemiddle.setLinearHeadingInterpolation(scoresampleinbasketpose.getHeading(), samplemiddlepose.getHeading());
//                setPathState(118);
//                break;
//            case 118:
//                if(pathTimer.getElapsedTimeSeconds() > 3.5){
//                    follower.followPath(depositcycle2, false);
//                    setPathState(18);
//                }
//                break;
//            case 18:
//
//                depositcycle2.setLinearHeadingInterpolation(samplemiddlepose.getHeading(), scoresampleinbasketpose.getHeading());
//                setPathState(19);
//                break;
//            case 19:
//                if(pathTimer.getElapsedTimeSeconds() > 4.2) {
//                    follower.followPath(goToSampleleft, false);
//                    setPathState(20);
//                }
//                break;
//            case 20:
//
//                goToSampleleft.setLinearHeadingInterpolation(scoresampleinbasketpose.getHeading(), sampleleftpose.getHeading());
//                setPathState(21);
//                break;
//            case 21:
//                if(pathTimer.getElapsedTimeSeconds() > 5){
//                    follower.followPath(depositcycle3, false);
//                    setPathState(22);
//                }
//                break;
//            case 22:
//
//                depositcycle3.setLinearHeadingInterpolation(sampleleftpose.getHeading(), scoresampleinbasketpose.getHeading());
//                setPathState(24);
//                break;

            case 24:
                if(!follower.isBusy()) {
                    setPathState(-1);
                    break;
                }

                // timeout!!!!!!!!!!!!
        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
//    public void autonomousActionUpdate() {
//        switch (actionState) {
//            case 0:
////                setClawState(0);
////                setActionState(-1);
//                break;
//            case 1:
////                setClawState(1);
////                setActionState(-1);
//                break;
//        }
//    }

    /** This switch is called continuously and runs the claw actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
//    public void clawUpdate() {
//        switch (clawState) {
//            case 0:
////                claw.groundClaw();
////                claw.closeClaws();
////                setClawState(-1);
//                break;
//            case 1:
////                claw.scoringClaw();
////                setClawState(-1);
//                break;
//        }
//    }
//    public void setActionState(int aState) {
//        actionState = aState;
//        pathTimer.resetTimer();
//        autonomousActionUpdate();
//    }
//
//    public void setClawState(int cState) {
//        clawState = cState;
//    }


    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();

    }
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
    }

    @Override
    public void init() {

        pathTimer = new Timer();
        opModeTimer = new Timer();
        scanTimer = new Timer();
        opModeTimer.resetTimer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        robot.init(hardwareMap);

    }

    @Override
    public void init_loop() {
// Camera code
    }
    @Override
    public void start() {
        setBackdropGoalPose();
        buildPaths();
        //30 sec auto timer
        opModeTimer.resetTimer();
        setPathState(10);
       // setActionState(0);
    }
}
