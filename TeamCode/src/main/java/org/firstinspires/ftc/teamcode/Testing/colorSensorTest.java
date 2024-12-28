package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.meet2.Arm;

@TeleOp(name="colorSensorTest", group="Test")
public class colorSensorTest extends LinearOpMode {
    Arm arm = new Arm();
     @Override
     public void runOpMode() {
         arm.init(hardwareMap);

         waitForStart();

         while (opModeIsActive()) {
             telemetry.addData("Color Sensor Distance", arm.getSpecimenColorSensor());
             telemetry.update();
         }
     }
}
