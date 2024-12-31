package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

//package org.firstinspires.ftc.robotcontroller.external.samples;


/*
 * This OpMode demonstrates how to use a REV Robotics Touch Sensor, REV Robotics Magnetic Limit Switch, or other device
 * that implements the TouchSensor interface. Any touch sensor that connects its output to ground when pressed
 * (known as "active low") can be configured as a "REV Touch Sensor". This includes REV's Magnetic Limit Switch.
 *
 * The OpMode assumes that the touch sensor is configured with a name of "sensor_touch".
 *
 * A REV Robotics Touch Sensor must be configured on digital port number 1, 3, 5, or 7.
 * A Magnetic Limit Switch can be configured on any digital port.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "azDigitalTest", group = "Test")
@Disabled
public class azDigitalTest extends LinearOpMode {
    DigitalChannel extensionTouch = null;
    DigitalChannel pivotTouch = null;// Touch sensor Object
    //public DcMotor  arm     = null;

    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        //arm   = hardwareMap.get(DcMotor.class, "arm");

        extensionTouch = hardwareMap.get(DigitalChannel.class, "extensionTouch");
        pivotTouch = hardwareMap.get(DigitalChannel.class, "pivotTouch");

        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop conditiont because it is an interruptible method.
        while (opModeIsActive()) {
                telemetry.addData("Extension Touch", extensionTouch.getState());
                telemetry.addData("PivotTouch", pivotTouch.getState());

            //arm.setPower(1);

                // send the info back to driver station using telemetry function.


            telemetry.update();
        }
    }
}
