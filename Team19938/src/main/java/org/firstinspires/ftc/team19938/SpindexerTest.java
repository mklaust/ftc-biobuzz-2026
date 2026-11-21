package org.firstinspires.ftc.team19938;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.sun.source.doctree.StartElementTree;

@TeleOp(name = "Spindexer Test")
public class SpindexerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setVelocityPIDFCoefficients(5.46, 0.55, 0.0, 54.6);
        motor.setPositionPIDFCoefficients(5.0);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int target = 0;
        boolean lastA = false;

        waitForStart();

        while (opModeIsActive()) {
            boolean currA = gamepad1.a;
            if (currA && !lastA) {
                target += 288;
                motor.setTargetPosition(target);
                motor.setPower(0.8);
            }

            lastA = currA;

            telemetry.addData("Target", target);
            telemetry.addData("Current", motor.getCurrentPosition());
            telemetry.addData("Power", motor.getPower());
            telemetry.addData("Pos PIDF", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
            telemetry.addData("Vel PIDF", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }
}
