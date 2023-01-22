package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PowerPlay.PowerPlayRobot;

@TeleOp
public class PotentiometerTest extends LinearOpMode {
    PowerPlayRobot r2;

    //This function is executed when this Op Mode is selected from the Driver Station.
    @Override
    public void runOpMode() {

        // Declaring the former values of the buttons, so we can tell if they changed.
        // Note: Currently no such variables

        // r2 has arrived.
        r2 = new PowerPlayRobot(this);
        r2.initHardware();        // Get the potentiometer and motor from hardwareMap

        AnalogInput potentiometer;

        // Define variable for the current voltage
        double currentVoltage;

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Get the potentiometer and motor from hardwareMap
            potentiometer = hardwareMap.get(AnalogInput.class, "LiftAngleSensor");

            // Loop while the Op Mode is running
            waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("Potentiometer voltage", potentiometer.getVoltage());
                telemetry.update();
            }
        }
    }
}