package org.firstinspires.ftc.teamcode.TeleOp.meet1;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Config
//@TeleOp

public class armpidtuning extends OpMode {
    private PIDController controller;

    public static double p = 0.0035, i = 0, d = 0.00001;
    public static double f = 0.00005;
    public static int target = 0;
    private final double ticks_in_degree = 700 / 180.0;
    private DcMotorEx arm_motor;
    @Override
    public void init(){

        controller = new PIDController(p,i,d);
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
    arm_motor.setDirection(DcMotor.Direction.REVERSE);

    }
    @Override
    public void loop() {
 controller.setPID(p,i,d);
    int armPos = (arm_motor.getCurrentPosition());
    double pid = controller.calculate(armPos, target);
    double ff = Math.cos(Math. toRadians(target / ticks_in_degree)) * f;

    double power = pid+ff;
    arm_motor.setPower(power);

    telemetry.addData(" pos",armPos);
    telemetry.addData(" pos",armPos);

}

}
