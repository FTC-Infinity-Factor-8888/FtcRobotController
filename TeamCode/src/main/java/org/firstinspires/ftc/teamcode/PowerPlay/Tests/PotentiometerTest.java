package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

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
        CRServo motor;

        // Define variable for the current voltage
        double currentVoltage;

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // Get the potentiometer and motor from hardwareMap
            potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
            motor = hardwareMap.get(CRServo.class, "liftMotor");

            // Loop while the Op Mode is running
            waitForStart();
            while (opModeIsActive()) {
                // Update currentVoltage from the potentiometer
                currentVoltage = potentiometer.getVoltage();

                // Turn the motor on or off based on the potentiometer’s position
                if (currentVoltage < 1.65) {
                    motor.setPower(0);
                } else {
                    motor.setPower(0.3);
                }

                // Show the potentiometer’s voltage in telemetry
                telemetry.addData("Potentiometer voltage", currentVoltage);
                telemetry.update();
            }
        }
    }
}