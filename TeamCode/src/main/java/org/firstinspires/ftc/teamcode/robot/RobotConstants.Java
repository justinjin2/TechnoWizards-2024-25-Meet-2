package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;

@Config
public class RobotConstants {
    public static int
    //LIFT CONSTANTS
// max extension for scoring slides
LIFT_MAX_POS = 4735,
   


    public static final int
    //Mechanism States
INTAKE_IN = 0,
INTAKE_DONE = 1,
OUTTAKE_START = 2,

    public static double
    //Servo positions

    /**
    * Times are in miliseconds
    */
    public static long
    //Time it takes for stuff

 public static CustomPIDFCoefficients

 // liftPIDF coefficients
  liftPIDFCoefficients = new CustomPIDFCoefficients(
      0
      0,
      0,
      0);
}