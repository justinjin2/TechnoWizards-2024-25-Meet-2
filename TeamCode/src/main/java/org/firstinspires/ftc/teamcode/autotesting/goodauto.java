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
    public int actionState, clawState;
    ElapsedTime waitTimer1 = new ElapsedTime();

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
    private Pose preloadspecimanpose, pullspecimanpose,samplerightpose, samplemiddlepose, sampleleftpose, scoresampleinbasketpose;

    private Pose startPose = new Pose(137, 41, Math.toRadians(270));

    private Follower follower;
    public armandintake sub;

    private Path specimanPreloaddeposit,  pullspeciman, goToSampleright, goToSamplemiddle, goToSampleleft, depositcycle1, depositcycle2, depositcycle3;

    private int pathState;

    //define team element prop and create paths based on the case seen
    public void setBackdropGoalPose(){
        preloadspecimanpose = new Pose(115, 64, Math.toRadians(0));
       pullspecimanpose = new Pose(120, 64, Math.toRadians(0));


    }
    public void buildPaths() {

        specimanPreloaddeposit = new Path(new BezierLine(new Point(startPose), new Point(preloadspecimanpose)));
        pullspeciman = new Path(new BezierLine(new Point(preloadspecimanpose), new Point(pullspecimanpose)));


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
                sub.armspecpos();
                specimanPreloaddeposit.setLinearHeadingInterpolation(startPose.getHeading(),preloadspecimanpose.getHeading());

                setPathState(13);
                break;
            case 13:
                if(!follower.isBusy()) {

                    follower.followPath(pullspeciman);
                }
                setPathState(14);
                break;
            case 14:
                pullspeciman.setConstantHeadingInterpolation(pullspecimanpose.getHeading());
                sub.armspecppull();

                setPathState(15);
                break;

            case 15:
                if(!follower.isBusy()) {
                    setPathState(13);
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
