package org.firstinspires.ftc.teamcode.autotesting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierPoint;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "test1", group = "test" )
public class test extends OpMode {
    private Timer pathTimer, opModeTimer, scanTimer;

    ElapsedTime waitTimer1 = new ElapsedTime();

    //Spike Mark Poses


//Backdrop Poses

//White stack Poses

    private Pose preloadspecimanpose, samplerightpose, samplemiddlepose, sampleleftpose, scoresampleinbasketpose;

    private Pose startPose = new Pose(137, 90, Math.toRadians(180));

    private Follower follower;

    private Path specimanPreloaddeposit, goToSampleright, goToSamplemiddle, goToSampleleft, depositcycle1, depositcycle2, depositcycle3;

    private int pathState;

    //define team element prop and create paths based on the case seen
    public void setBackdropGoalPose(){
//switch(navigation){
//    default:
//    case "left":
//    spikeMarkGoalPose = new Pose(blueLeftSideLeftSpikeMark.getX() + 0.5, blueLeftSideLeftSpikeMark.getY(), Math.PI/2);
//    break;
//    case "middle":
//    spikeMarkGoalPose = new Pose(blueLeftSideMiddleSpikeMark.getX(), blueLeftSideMiddleSpikeMark.getY()+3, Math.PI/2);
//    break;
//    case "right":
//    spikeMarkGoalPose = new Pose(blueLeftSideRightSpikeMark.getX(), blueLeftSideRightSpikeMark.getY(), Math.PI/2);
//    break;

// Left


        preloadspecimanpose = new Pose(110, 69, Math.toRadians(180));





    }
    public void buildPaths() {


        specimanPreloaddeposit = new Path(new BezierLine(new Point(startPose), new Point(preloadspecimanpose)));


//firstcyclebackdrop = new Path(new BezierCurve(new Point(firstCycleStackPose), new Point(1,2, Point.CARTESIAN)))
    }

//cycleToBackboard = follower.pathBuilder()

    //.build();
    public void autonomousPathUpdate(){
        switch(pathState){
            case 10: //First Scored Path
                follower.followPath(specimanPreloaddeposit);
                //When starting to move instantly moves to case 11
                setPathState(11);
                break;
            case 11: //While moving check how far away from wall and start interpolating

                specimanPreloaddeposit.setConstantHeadingInterpolation(preloadspecimanpose.getHeading());
                setPathState(24);
                break;

            case 24:
                if(!follower.isBusy()) {
                    setPathState(-1);
                    break;
                }

                // timeout!!!!!!!!!!!!
        }
    }
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
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        scanTimer = new Timer();
        opModeTimer.resetTimer();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
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
    }
}
