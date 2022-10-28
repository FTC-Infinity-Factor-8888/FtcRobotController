package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="CalculatePIDF")
public class CalculateWheelPIDFValue extends LinearOpMode {
    DcMotorEx lfMotor;
    DcMotorEx rfMotor;
    DcMotorEx lrMotor;
    DcMotorEx rrMotor;
    double currentVelocity;
    double lfMaxVelocity = 0.0;
    double rfMaxVelocity = 0.0;
    double lrMaxVelocity = 0.0;
    double rrMaxVelocity = 0.0;



    @Override
    public void runOpMode() {
        lfMotor = hardwareMap.get(DcMotorEx.class, "LFMotor");
        lrMotor = hardwareMap.get(DcMotorEx.class, "LRMotor");
        rfMotor = hardwareMap.get(DcMotorEx.class, "RFMotor");
        rrMotor = hardwareMap.get(DcMotorEx.class, "RRMotor");

        lfMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lrMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (opModeIsActive()) {
            lfMotor.setPower(1.0);
            rfMotor.setPower(1.0);
            lrMotor.setPower(1.0);
            rrMotor.setPower(1.0);

            int count = 1;

            int averageCount = 0;
            double pAverage = 0;
            double iAverage = 0;
            double fAverage = 0;

            while (opModeIsActive()) {
                currentVelocity = lfMotor.getVelocity();
                if (currentVelocity > lfMaxVelocity) {
                    lfMaxVelocity = currentVelocity;
                }

                currentVelocity = rfMotor.getVelocity();

                if (currentVelocity > rfMaxVelocity) {
                    rfMaxVelocity = currentVelocity;
                }
                currentVelocity = lrMotor.getVelocity();
                if (currentVelocity > lrMaxVelocity) {
                    lrMaxVelocity = currentVelocity;
                }

                currentVelocity = rrMotor.getVelocity();
                if (currentVelocity > rrMaxVelocity) {
                    rrMaxVelocity = currentVelocity;
                }

                double P;
                double I;
                double D = 0;
                double F;

                if(count >= 10) {
                    F = 32767 / lfMaxVelocity;
                    P = 0.1 * F;
                    I = 0.1 * P;

                    pAverage += P;
                    iAverage += I;
                    fAverage += F;
                    averageCount++;

                    telemetry.addData("LF", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                    System.out.printf("LF - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, lfMaxVelocity);


                    F = 32767 / rfMaxVelocity;
                    P = 0.1 * F;
                    I = 0.1 * P;

                    pAverage += P;
                    iAverage += I;
                    fAverage += F;
                    averageCount++;

                    telemetry.addData("RF", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                    System.out.printf("RF - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, rfMaxVelocity);


                    F = 32767 / lrMaxVelocity;
                    P = 0.1 * F;
                    I = 0.1 * P;

                    pAverage += P;
                    iAverage += I;
                    fAverage += F;
                    averageCount++;

                    telemetry.addData("LR", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                    System.out.printf("LR - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, lrMaxVelocity);

                    F = 32767 / rrMaxVelocity;
                    P = 0.1 * F;
                    I = 0.1 * P;

                    pAverage += P;
                    iAverage += I;
                    fAverage += F;
                    averageCount++;

                    telemetry.addData("RR", "P (%.2f) I (%.3f) D (%.3f) F (%.1f)", P, I, D, F);
                    System.out.printf("RR - P (%.2f) I (%.3f) D (%.3f) F (%.1f) TPS (%.0f)\n", P, I, D, F, rrMaxVelocity);
                }
                telemetry.update();
                count++;
                System.out.printf("AVERAGE - P (%.2f) I (%.3f) D (%.3f) F (%.1f)\n", pAverage/averageCount, iAverage/averageCount, 0.0, fAverage/averageCount);
            }
        }
    }
}

