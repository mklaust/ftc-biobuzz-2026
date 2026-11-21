/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.team19938;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates how to program your robot to drive field relative.  This means
 * that the robot drives the direction you push the joystick regardless of the current orientation
 * of the robot.
 *
 * This OpMode assumes that you have four mecanum wheels each on its own motor named:
 *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
 *
 *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 */
@TeleOp(name = "TeleOp-19938", group = "Robot")
//@Disabled
public class BioBuzzTeleOp19938 extends OpMode {
    // This declares the four motors needed
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor feederRight;
    private DcMotor feederLeft;
    private DcMotorEx launcher;
    private DcMotor spindexer;
    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private SpindexerIndexer spindexerIndexer;
    private DirectionalIntaking directionalIntaking;
    private SpindexerState spindexerState;
    private int spindexerPos;
    private static final double BALL_THRESHOLD = 10.0;
    private static final int TICKS_PER_FULL_360 = 288;
    private static final double FLUSH_SPEED = 0.7;
    private static final int TICKS_PER_SPINDEXER_SLOT = 58;
    private static final double INDEX_SPEED = 0.3;


    public enum SpindexerState {
        INDEXING, // lets one in and spins to an empty spot so it can let another in, will have to integrate with directional intaking
        FIRING,   // spins to fireballs, also might be good to integrate with directional
        FILLED,   // leaves empty space in front of launcher,
        IDLE      // does nothing, nulls intake - also should have manual rotation function button of 1/5
    }

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        feederRight = hardwareMap.get(DcMotor.class, "frontFeed");
        feederLeft = hardwareMap.get(DcMotor.class, "backFeed");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        spindexer = hardwareMap.get(DcMotor.class, "spindexer");

        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "color_sensor_right");
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "color_sensor_left");

        distanceSensorRight.resetDeviceConfigurationForOpMode();
        distanceSensorLeft.resetDeviceConfigurationForOpMode();


        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        imu = hardwareMap.get(IMU.class, "IMU");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        spindexerIndexer = new SpindexerIndexer();
        directionalIntaking = new DirectionalIntaking(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                feederRight,
                feederLeft
        );

        spindexerState = SpindexerState.IDLE;
        spindexerPos = 0;
    }

    @Override
    public void loop() {

        telemetry.addData("spindexerPos", spindexerPos);
        telemetry.addData("spindexer state", spindexerState);
        telemetry.addData("ball count", spindexerIndexer.getBallCount());
        telemetry.addData("Left distance (cm): ", "%.3f", distanceSensorLeft.getDistance(DistanceUnit.CM));
        telemetry.addData("right distance (cm): ", "%.3f", distanceSensorRight.getDistance(DistanceUnit.CM));

        launcher.setVelocity(2000);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }

        directionalIntaking.update(gamepad1, spindexerIndexer);

        boolean ballDetectedRight = distanceSensorRight.getDistance(DistanceUnit.CM) < BALL_THRESHOLD;
        boolean ballDetectedLeft = distanceSensorLeft.getDistance(DistanceUnit.CM) < BALL_THRESHOLD;
        boolean launchRequested = gamepad1.left_bumper;

        switch (spindexerState) {
            case INDEXING:
                // Avoid firing while we're indexing to make sure we don't get out of sync
                if (!spindexer.isBusy()) {
                    spindexerState = SpindexerState.IDLE;
                }
                break;
            case FIRING:
                if (!spindexer.isBusy()) {
                    spindexer.setPower(0.0);
                    //spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexerIndexer.resetTracker();
                    spindexerState = SpindexerState.IDLE;
                }
                break;
            case FILLED:
                if (launchRequested) {
                    launch();
                    spindexerState = SpindexerState.FIRING;
                }
                break;
            case IDLE:
                if (launchRequested) {
                    launch();
                    spindexerState = SpindexerState.FIRING;
                }
                else if (spindexerIndexer.isFull()) {
                    spindexerState = SpindexerState.FILLED;
                }
                else if (ballDetectedRight) {
                    spindexerIndexer.indexBallClockwise();
                    if (spindexerIndexer.isFull()) {
                        spindexerState = SpindexerState.FILLED;
                    }
                    else {
                        chamberIntake();
                        spindexerState = SpindexerState.INDEXING;
                    }
                }

                break;
        }





        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
//        if (gamepad1.left_bumper) {
//            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        } else {
//            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//        }

        telemetry.update();
    }

    private void launch() {
        spindexerPos = spindexer.getCurrentPosition() + TICKS_PER_FULL_360;
        spindexer.setTargetPosition(spindexerPos);
        //spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(FLUSH_SPEED);
    }

    private void chamberIntake(){
        spindexerPos = spindexer.getCurrentPosition() - TICKS_PER_SPINDEXER_SLOT;
        spindexer.setTargetPosition(spindexerPos);
        //spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setPower(INDEX_SPEED);
    }

// sensors named this \/
    //color_sensor_right/left

    // This routine drives the robot field relative
//    private void driveFieldRelative(double forward, double right, double rotate) {
//        // First, convert direction being asked to drive to polar coordinates
//        double theta = Math.atan2(forward, right);
//        double r = Math.hypot(right, forward);
//
//        // Second, rotate angle by the angle the robot is pointing
//        theta = AngleUnit.normalizeRadians(theta -
//                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
//
//        // Third, convert back to cartesian
//        double newForward = r * Math.sin(theta);
//        double newRight = r * Math.cos(theta);
//
//        // Finally, call the drive method with robot relative forward and right amounts
//        drive(newForward, newRight, rotate);
//    }

    //spindexer procedures \/


    // Thanks to FTC16072 for sharing this code!!
//    public void drive(double forward, double right, double rotate) {
//        // This calculates the power needed for each wheel based on the amount of forward,
//        // strafe right, and rotate
//        double frontLeftPower = forward + right + rotate;
//        double frontRightPower = forward - right - rotate;
//        double backRightPower = forward + right - rotate;
//        double backLeftPower = forward - right + rotate;
//
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;  // make this slower for outreaches
//
//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//
//        // We multiply by maxSpeed so that it can be set lower for outreaches
//        // When a young child is driving the robot, we may not want to allow full
//        // speed.
//        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
//        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
//        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
//        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
//    }
}
