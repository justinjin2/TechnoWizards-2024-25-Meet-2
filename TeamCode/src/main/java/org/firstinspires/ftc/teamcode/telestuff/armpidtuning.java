package org.firstinspires.ftc.teamcode.telestuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.util.FilteredPIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.robot.robot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDcontroller;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class armpidtuning extends OpMode {
    private PIDFController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;
    private DcMotorEx arm_motor;
    @Override
    public void init(){

        controller = new PIDFController(p,i,d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");

    }
    @Override
    public void loop() {
//controller.setCoefficients();
    int armPos = (arm_motor.getCurrentPosition());
    double pid = controller.calculate(armPos, target);
    double ff = Math.cos(Math. toRadians(target / ticks_in_degree)) * f;

    double power = pid+ff;
    arm_motor.setPower(power);

    telemetry.addData(" pos",armPos);
    telemetry.addData(" pos",armPos);

}

}
