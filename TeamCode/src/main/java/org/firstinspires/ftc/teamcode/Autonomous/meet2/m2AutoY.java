package org.firstinspires.ftc.teamcode.Autonomous.meet2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.meet1.armandintake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous (name = "1plus3", group = "test" )
public class m2AutoY extends OpMode {
    private Timer pathTimer, opModeTimer, scanTimer;
    // robot robot = new robot();
    public int actionState, clawState;
    ElapsedTime waitTimer1 = new ElapsedTime();
    //waittime1
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
    private Pose preloadspecimanpose, pullspecimanpose, samplerightpose, scooprightsamplepose,samplemiddlepose, sampleleftpose, sampleleftsweeppose, scoresampleinbasketpose,scoresampleinbasketpose2, parkpose;

    private Pose startPose = new Pose(137, 41, Math.toRadians(270));

    private Follower follower;
    public armandintake sub;

    private Path specimanPreloaddeposit, park, pullspeciman, goToSampleright, goTosampleleftsweep,scooprightsample,goToSamplemiddle, goToSampleleft, depositcycle1, depositcycle2, depositcycle3;

    private int pathState;

    //define team element prop and create paths based on the case seen
    public void setBackdropGoalPose(){
        preloadspecimanpose = new Pose(115, 64, Math.toRadians(0));
        pullspecimanpose = new Pose(125, 64, Math.toRadians(0));
        samplerightpose = new Pose(103, 44, Math.toRadians(70));
        scooprightsamplepose = new Pose(108, 44, Math.toRadians(65));
        scoresampleinbasketpose = new Pose(117.5, 23, Math.toRadians(140));
        samplemiddlepose = new Pose(109, 33, Math.toRadians(65));
        scoresampleinbasketpose2 = new Pose(120, 21.5, Math.toRadians(140));
        sampleleftpose = new Pose(102, 28.5, Math.toRadians(85));
        sampleleftsweeppose = new Pose(101.3, 25.6, Math.toRadians(85));
        parkpose = new Pose(75, 45, Math.toRadians(270));

    }
    public void buildPaths() {
        Point tosamplerightmidpoint;
        tosamplerightmidpoint = new Point(115, 59, Point.CARTESIAN);
        Point tosamplerightmidpoint2;
        tosamplerightmidpoint2 = new Point(100, 38, Point.CARTESIAN);
       // tosamplerightmidpoint = new Point(128, 50, Point.CARTESIAN);
        Point middlescoremidpoint;
        middlescoremidpoint = new Point(124, 43, Point.CARTESIAN);
        Point leftscoremidpoint;
        leftscoremidpoint = new Point(110, 40, Point.CARTESIAN);


        // tosamplemiddlemidpoint = new Point(95, 31, Point.CARTESIAN);

        specimanPreloaddeposit = new Path(new BezierLine(new Point(startPose), new Point(preloadspecimanpose)));
        pullspeciman = new Path(new BezierLine(new Point(preloadspecimanpose), new Point(pullspecimanpose)));
        goToSampleright = new Path(new BezierCurve(new Point(pullspecimanpose), tosamplerightmidpoint,new Point(samplerightpose)));
        // scooprightsample = new Path(new BezierLine(new Point(samplerightpose), new Point(scooprightsamplepose)));
        depositcycle1 = new Path(new BezierLine(new Point(samplerightpose), new Point(scoresampleinbasketpose)));
        goToSamplemiddle = new Path(new BezierLine(new Point(scoresampleinbasketpose),new Point(samplemiddlepose)));
        depositcycle2 = new Path(new BezierCurve(new Point(samplemiddlepose), middlescoremidpoint, new Point(scoresampleinbasketpose2)));
        goToSampleleft = new Path(new BezierLine(new Point(scoresampleinbasketpose2),new Point(sampleleftpose)));
        goTosampleleftsweep = new Path(new BezierLine(new Point(sampleleftpose),new Point(sampleleftsweeppose)));
        depositcycle3 = new Path(new BezierCurve(new Point(sampleleftsweeppose), leftscoremidpoint, new Point(scoresampleinbasketpose2)));
       park = new Path(new BezierLine(new Point(scoresampleinbasketpose2),new Point(parkpose)));

//firstcyclebackdrop = new Path(new BezierCurve(new Point(firstCycleStackPose), new Point(1,2, Point.CARTESIAN)))
    }

//cycleToBackboard = follower.pathBuilder()

    //.build();
    public void autonomousPathUpdate(){
        switch(pathState){
            case 10: //First Scored Path
                follower.followPath(specimanPreloaddeposit,false);

                //When starting to move instantly moves to case 11
                setPathState(11);
                break;
            case 11: //While moving check how far away from wall and start interpolating
                sub.armspecpos();
                specimanPreloaddeposit.setLinearHeadingInterpolation(startPose.getHeading(),preloadspecimanpose.getHeading());

                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy()) {

                    follower.followPath(pullspeciman,false);
                    setPathState(14);
                }

                break;
            case 14:
                pullspeciman.setConstantHeadingInterpolation(pullspecimanpose.getHeading());
                sub.armspecppull();

                setPathState(16);
                break;
            case 16: //First Scored Path
                if(!follower.isBusy()) {
                    follower.followPath(goToSampleright, false);
                    setPathState(17);
                }
                //When starting to move instantly moves to case 11

                break;
            case 17: //While moving check how far away from wall and start interpolating

                goToSampleright.setLinearHeadingInterpolation(pullspecimanpose.getHeading(), samplerightpose.getHeading(),0.3);
                waitTimer1.reset();
                setPathState(18);
                break;
            case 18: //While moving check how far away from wall and start interpolating
                // if(follower.getCurrentTValue() > 0.1) {
                if(waitTimer1.seconds() >= 0.4) {
                sub.armintakeon();
                waitTimer1.reset();
                setPathState(21);
                 }
//
                break;
            case 21: //While moving check how far away from wall and start interpolating

                if(!follower.isBusy() && waitTimer1.seconds() >= 1) {

                    sub.armsampledepopos();
                    setPathState(22);
                }

                break;
            case 22: //First Scored Path

                follower.followPath(depositcycle1, false);
                setPathState(23);

                //When starting to move instantly moves to case 11

                break;
            case 23: //While moving check how far away from wall and start interpolating
                sub.stopintake();
                depositcycle1.setLinearHeadingInterpolation(scooprightsamplepose.getHeading(),scoresampleinbasketpose.getHeading());

                setPathState(24);
                break;
            case 24: //While moving check how far away from wall and start interpolating
                if(!follower.isBusy()) {
                    sub.outtake();
                    waitTimer1.reset();
                    setPathState(25);
                }


                break;
            case 25: //First Scored Path
                if(waitTimer1.seconds() >= 0.5) {
                    follower.followPath(goToSamplemiddle, false);

                    setPathState(26);
                }

                break;
            case 26: //While moving check how far away from wall and start interpolating

                goToSamplemiddle.setLinearHeadingInterpolation(scoresampleinbasketpose.getHeading(), samplemiddlepose.getHeading());

                setPathState(27);
                break;
            case 27: //While moving check how far away from wall and start interpolating
                if(follower.getCurrentTValue() > 0.3) {
                    sub.armintakeon();
                    waitTimer1.reset();
                    setPathState(28);
                }

                break;
            case 28: //First Scored Path
                if(!follower.isBusy() && waitTimer1.seconds() >= 1.5) {

                    sub.armsampledepopos();
                    follower.followPath(depositcycle2, false);
                    setPathState(30);
                }


                //When starting to move instantly moves to case 11

                break;

            case 30: //While moving check how far away from wall and start interpolating
                sub.stopintake();
                depositcycle2.setLinearHeadingInterpolation(samplemiddlepose.getHeading(),scoresampleinbasketpose2.getHeading());

                setPathState(31);
                break;
            case 31: //While moving check how far away from wall and start interpolating
                if(!follower.isBusy()) {
                    sub.outtake();
                    waitTimer1.reset();
                    setPathState(32);
                }
                break;
            case 32: //First Scored Path
                if(waitTimer1.seconds() >= 0.5) {
                    follower.followPath(goToSampleleft, false);

                    setPathState(33);
                }

                break;
            case 33: //While moving check how far away from wall and start interpolating

                goToSampleleft.setLinearHeadingInterpolation(scoresampleinbasketpose2.getHeading(), sampleleftpose.getHeading());

                setPathState(34);
                break;
            case 34: //While moving check how far away from wall and start interpolating
                if(follower.getCurrentTValue() > 0.3) {
                    sub.armintakeon();
                    waitTimer1.reset();
                    setPathState(35);
                }
            case 35: //First Scored Path
                if(!follower.isBusy()) {
                    follower.followPath(goTosampleleftsweep, false);

                    setPathState(36);
                }

                break;
            case 36: //While moving check how far away from wall and start interpolating

                goTosampleleftsweep.setConstantHeadingInterpolation(sampleleftsweeppose.getHeading());
                waitTimer1.reset();
                setPathState(37);
                break;
            case 37: //First Scored Path
                if(!follower.isBusy() && waitTimer1.seconds() >= 1) {

                    sub.armsampledepopos();
                    follower.followPath(depositcycle3, false);
                    setPathState(38);
                }
                break;
            case 38: //While moving check how far away from wall and start interpolating
                sub.stopintake();
                depositcycle3.setLinearHeadingInterpolation(sampleleftpose.getHeading(),scoresampleinbasketpose2.getHeading());
                setPathState(39);

                break;
            case 39: //While moving check how far away from wall and start interpolating
                if(!follower.isBusy()) {
                    sub.outtake();
                    waitTimer1.reset();
                    setPathState(40);
                }
                break;
            case 40: //First Scored Path
                if(!follower.isBusy() && waitTimer1.seconds() >= 0.5) {
                    follower.followPath(park, false);

                    setPathState(41);
                }

                break;
            case 41: //While moving check how far away from wall and start interpolating
                sub.stopintake();
                park.setLinearHeadingInterpolation(scoresampleinbasketpose2.getHeading(),parkpose.getHeading(),0.3);

                setPathState(15);
                break;


            case 15:
                if(!follower.isBusy()) {
                    setPathState(-1);
                    break;
                }

                // timeout!!!!!!!!!!!!
        }
    }

    public void autonomousActionUpdate() {

        switch (actionState) {
            case 0:
                setClawState(0);
                setActionState(-1);
                break;
            case 1:
                setClawState(1);
                setActionState(-1);
                break;
        }
    }

    /** This switch is called continuously and runs the claw actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    public void clawUpdate() {
        switch (clawState) {
            case 0:
                sub.armMotor.setPower(1);
                sub.armMotor.setTargetPosition(SPECARMPOS);
                sub. armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sub.wrist.setPosition(1);
                setClawState(1);
                break;
            case 1:

                setClawState(-1);
                break;
        }
    }

    public void setPathState(int state){
        pathState = state;
        pathTimer.resetTimer();
        autonomousPathUpdate();

    }
    public void setActionState(int aState) {
        actionState = aState;
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }

    public void setClawState(int cState) {
        clawState = cState;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        autonomousActionUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("percentage of complete path:", (follower.getCurrentTValue() * 100));
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
