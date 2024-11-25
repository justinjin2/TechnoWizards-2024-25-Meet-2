package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;

@Config
public class RobotConstants {
    public static int
    //LIFT CONSTANTS
// max extension for scoring slides
    SPECIMAN_INTAKE = -4600,
    LOW_BUCKET = -2800,
    SPECIMAN_SCORING = -2750,
    SPECIMAN_PULLDOWN = -3200,
    LEVEL_TWO_HANG_START = -2450,
          STATIONARYPOS = -400,
    LEVEL_TWO_HANG_END = 200 ;//Should be 0, Check it later


    public static final int
    //Mechanism States
INTAKE_READY =0,
INTAKE_IN = 1,
INTAKE_DONE = 2,
OUTTAKE_START = 3,
OUTTAKE_DEPOSIT = 4;


    public static double
    //Servo positions
    WRISTCENTERPOS = 0.512,
    WRISTSPECPOS = 1,
    INTAKEON = 0,
   INTAKEOUT = 1,
            INTAKEOFF = 0.5;

    /**
    * Times are in miliseconds
    */
    public static long
            INTAKE_TO_BUCKET=550,
            INTAKE_TO_SPECIMEN=575,
            SPECIMEN_SCORING_PULLDOWN= 330,
            INTAKE_TO_HANGSTART= 850,
            HANGSTART_TO_HANGEND= 1750;
    //Time it takes for stuff

 public static CustomPIDFCoefficients

 // liftPIDF coefficients
  liftPIDFCoefficients = new CustomPIDFCoefficients(
      0.0035,
      0,
      0.00001,
      0.00005);
}