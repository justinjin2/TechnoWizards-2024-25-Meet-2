package org.firstinspires.ftc.teamcode.autotesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.robot.armandintake;
import org.firstinspires.ftc.teamcode.robot.robot;

@Autonomous (name = "1plus3", group = "test" )
public class goodauto extends OpMode {
    private Timer pathTimer, opModeTimer, scanTimer;
   // robot robot = new robot();

    ElapsedTime waitTimer1 = new ElapsedTime();


    private Pose preloadspecimanpose, samplerightpose, samplemiddlepose, sampleleftpose, scoresampleinbasketpose;

    private Pose startPose = new Pose(137, 41, Math.toRadians(270));

    private Follower follower;
    public armandintake sub;

    private Path specimanPreloaddeposit, goToSampleright, goToSamplemiddle, goToSampleleft, depositcycle1, depositcycle2, depositcycle3;

    private int pathState;

    //define team element prop and create paths based on the case seen
    public void setBackdropGoalPose(){
        preloadspecimanpose = new Pose(110, 62, Math.toRadians(0));

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

                specimanPreloaddeposit.setLinearHeadingInterpolation(startPose.getHeading(),preloadspecimanpose.getHeading(),0.5);
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
        sub = new armandintake(hardwareMap);
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
