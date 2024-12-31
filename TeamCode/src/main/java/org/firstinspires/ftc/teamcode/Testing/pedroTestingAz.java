package org.firstinspires.ftc.teamcode.Testing;


import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Disabled
@Autonomous (name = "pedro test", group = "test" )
public class pedroTestingAz extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private final Pose startPose = new Pose (135, 89, Math.toRadians(0));

    private Pose scorePose = new Pose (118, 80, Math.toRadians(0));
    private Pose wallIntakePose = new Pose(127.5, 116, Math.toRadians(0));
    private Pose pushPickup1ReadyPose = new Pose(85,118, Math.toRadians(0));
    private Pose pickup1ControlPose1 = new Pose (122, 107, Math.toRadians(0));
    private Pose pickup1ControlPose2 = new Pose (100, 105, Math.toRadians(0));
    private Pose endPushPose1 =new Pose(128,pushPickup1ReadyPose.getY(), Math.toRadians(0));

    private Pose pushPickup2ReadyPose = new Pose(83,127, Math.toRadians(0));
    private Pose endPushPose2 =new Pose(125,pushPickup2ReadyPose.getY(), Math.toRadians(0));
    private Pose pickup2ControlPose = new Pose(112,102, Math.toRadians(0));

    private Pose pushPickup3ReadyPose = new Pose(83,135.5, Math.toRadians(0));
    private Pose endPushPose3 =new Pose(128,pushPickup3ReadyPose.getY(), Math.toRadians(0));
    private Pose pickup3ControlPose = new Pose(112,112, Math.toRadians(0));

    private Pose parkPose = new Pose (125, 125, Math.toRadians(0));

    private int pathState;

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, wallIntake1, wallToScore, park;
    private PathChain pushPickup1Ready, pushPickup1, pushPickup2Ready, pushPickup2, pushPickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        wallIntake1 = new Path(new BezierLine(new Point(scorePose), new Point(wallIntakePose)));
        wallToScore = new Path(new BezierLine(new Point(wallIntakePose), new Point(scorePose)));

//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our pushPickup1 PathChain. We are using  a BezierCurve with 4 points, which is a curved line that is curved based off of the control point */
        pushPickup1= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup1ReadyPose), new Point(endPushPose1)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose1), new Point(endPushPose1.getX()-5, endPushPose1.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose1.getX()-5, endPushPose1.getY()), new Point(wallIntakePose)))
                .setConstantHeadingInterpolation(0)
                .build();

        pushPickup2= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
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

        pushPickup3= follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(pickup1ControlPose1), new Point(pickup1ControlPose2), new Point(pushPickup1ReadyPose)))
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
                .addPath(new BezierCurve(new Point(endPushPose2.getX()-5, endPushPose2.getY()), new Point(pickup3ControlPose), new Point(pushPickup3ReadyPose)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(pushPickup3ReadyPose), new Point(endPushPose3)))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose3), new Point(endPushPose3.getX()-5, endPushPose3.getY())))
                .setConstantHeadingInterpolation(0)
                .addPath(new BezierLine(new Point(endPushPose3.getX()-5, endPushPose3.getY()), new Point(wallIntakePose)))
                .setConstantHeadingInterpolation(0)
                .build();


//        park = new Path(new BezierCurve(new Point(starPose), /* Control Point */ new Point(parkPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());


    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: //First Scored Path
                telemetry.addData("run to", "preload score Pose");
                telemetry.update();
                follower.followPath(scorePreload, false);
                scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
                pathState=2;
                pathTimer.resetTimer();
                break;

            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5) {
                    telemetry.addData("run to", "wall Intake Pose");
                    telemetry.update();
                    follower.followPath(wallIntake1, false);
                    wallIntake1.setConstantHeadingInterpolation(0);
                    pathState=99;
                    pathTimer.resetTimer();
                }
                break;

            case 2:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5){
                    telemetry.addData("run to", "2nd score Pose");
                    telemetry.update();
                    follower.followPath(wallToScore, false);
                    wallToScore.setConstantHeadingInterpolation(0);
                    pathState=99;
                    pathTimer.resetTimer();
                }
                break;

            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5){
                    telemetry.addData("run to", "push 1 sample to human player and go to wall intake");
                    telemetry.update();
                    follower.followPath(pushPickup1, false);
                    pathState=99;
                    pathTimer.resetTimer();
                }
                break;

            case 20:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5){
                    telemetry.addData("run to", "push 2 samples to human player and go to wall intake");
                    telemetry.update();
                    follower.followPath(pushPickup2, false);
                    pathState=99;
                    pathTimer.resetTimer();
                }
                break;
            case 30:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>0.5){
                    telemetry.addData("run to", "push 3 samples to human player and go to wall intake");
                    telemetry.update();
                    follower.followPath(pushPickup3, false);
                    pathState=99;
                    pathTimer.resetTimer();
                }
                break;

            case 99:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds()>1) {
                    setPathState(100);
                }
                break;
            case 100:
                if(!follower.isBusy()) {
                    setPathState(-1);
                    break;
                }
        }
        telemetry.addData("pathState", pathState);
        telemetry.update();

    }

    public void setPathState ( int state){
        pathState = state;
        pathTimer.resetTimer();
    }

    /*********
     public void setAutoState(){
     pathTimer.resetTimer();
     }
     ****************/
    @Override
    public void loop () {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("percentage of complete path:", (follower.getCurrentTValue() * 100));
    }

    @Override
    public void init () {

        pathTimer = new Timer();
        opModeTimer = new Timer();

        opModeTimer.resetTimer();
        pathTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop () {
// Camera code
    }

    @Override
    public void start () {
        buildPaths();
        opModeTimer.resetTimer();
        setPathState(0);
    }
}