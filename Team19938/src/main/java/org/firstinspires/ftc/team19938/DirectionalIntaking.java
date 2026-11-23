package org.firstinspires.ftc.team19938;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BiobuzzTeleOp;

public class DirectionalIntaking {

    private static final double VELOCITY_THRESHOLD = 0.1; // Min drive threshold for auto-intake
    private static final double INTAKE_SPEED = 0.8;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final DcMotor feederRight;
    private final DcMotor feederLeft;

    private boolean dualInputEnabled = false; // Toggle for single vs dual input logic

    IMU imu;

    public DirectionalIntaking(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor fR, DcMotor fL, IMU imu) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.backLeft = bl;
        this.backRight = br;
        this.feederRight = fR;
        this.feederLeft = fL;
        this.imu = imu;
    }

    public void setDualInputEnabled(boolean enabled) {
        this.dualInputEnabled = enabled;
    }

    public boolean isDualInputEnabled() {
        return dualInputEnabled;
    }

    public void update(Gamepad driverGamepad, SpindexerIndexer tracker, boolean isSpindexerMoving) {
        double drive  = -driverGamepad.left_stick_y;
        double strafe =  driverGamepad.left_stick_x;
        double turn   =  driverGamepad.right_stick_x;


        double theta = Math.atan2(drive, strafe);
        double r = Math.hypot(strafe, drive);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newDrive = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);



        // 1. Mecanum Drive Calculation


        double flPower = newDrive + newStrafe + turn;
        double blPower = newDrive - newStrafe + turn;
        double frPower = newDrive - newStrafe - turn;
        double brPower = newDrive + newStrafe - turn;

        // Normalize motor powers
        double max = Math.max(Math.abs(flPower), Math.max(Math.abs(blPower),
                Math.max(Math.abs(frPower), Math.abs(brPower))));
        if (max > 1.0) {
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
        }

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);

        // 2. Automated Feeder Power Selection Logic based on drive direction
        double forwardVelocity = newDrive; // Velocity direction approximation from joystick input

        if (tracker.isFull() || isSpindexerMoving) {
            // Shut off all feeders when capacity (4 balls) is reached
            feederRight.setPower(0.0);
        //    feederLeft.setPower(0.0);
        } else if (forwardVelocity < VELOCITY_THRESHOLD) {
            // Moving forward toward Input A -> Enable Feeder A, disable Feeder B
            feederRight.setPower(-INTAKE_SPEED);
        //    feederLeft.setPower(0.0);
        } else if (forwardVelocity < -VELOCITY_THRESHOLD && dualInputEnabled) {
            // Moving backward toward Input B (if dual input active) -> Enable Feeder B, disable Feeder A
            feederRight.setPower(0.0);
        //    feederLeft.setPower(1.0);
        } else {
            // Stationary or strafing -> Keep both feeders off to prevent accidental entry/exit
            feederRight.setPower(0.0);
        //    feederLeft.setPower(0.0);
        }
    }
}
