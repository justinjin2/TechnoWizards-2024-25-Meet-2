package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/*
 * This OpMode demonstrates how to use a digital channel.
 *
 * The OpMode assumes that the digital channel is configured with a name of "digitalColor".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "TestColor", group = "Sensor")
@Disabled
public class azTestColor extends LinearOpMode {
    DigitalChannel digitalColor;  // Digital channel Object

    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        //digitalColor = hardwareMap.get(DigitalChannel.class, "digitalcolor");
        DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");


        // pin0.setMode(DigitalChannel.Mode.OUTPUT);
        //  pin1.setMode(DigitalChannel.Mode.OUTPUT);
        telemetry.addData("DigitalColorSensorExample", "output mode...");
        telemetry.update();


        // wait for the start button to be pressed.

        // while the OpMode is active, loop and read the digital channel.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("digital 0", pin0.getState());
            telemetry.addData("digital 1", pin1.getState());
            telemetry.update();
        }
    }
}


