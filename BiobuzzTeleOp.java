package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Biobuzz TeleOp Control System
 * 
 * Implements the 3-module software architecture for the FTC Biobuzz season:
 * 1. Virtual Carousel Slot Tracker (5-slot array, 4-ball limit memory tracker)
 * 2. Drivetrain & Automated Feeder Control (Mecanum drive + velocity-based feeder logic)
 * 3. Intake & Launch Finite State Machine (Non-blocking TeleOp loop)
 */
@TeleOp(name = "Biobuzz TeleOp Logic", group = "TeleOp")
public class BiobuzzTeleOp extends LinearOpMode {

    // =========================================================================
    // ENUMS AND TRACKER CLASSES
    // =========================================================================

    public enum RobotState {
        INTAKE_IDLE,
        INDEXING_BALL,
        CAPACITY_FULL,
        LAUNCHING_ALL
    }

    /**
     * Module 1: Virtual Carousel Slot Tracker
     * Tracks ball positions in memory using a 5-element circular array
     * since physical sensors are only located at the input ports.
     */
    public static class VirtualCarouselTracker {
        private final boolean[] slots = new boolean[5];
        public static final int CAROUSEL_CAPACITY = 4; // Capped strictly at 4 balls
        private int currentInputSlotIndex = 0;

        public int getBallCount() {
            int count = 0;
            for (boolean occupied : slots) {
                if (occupied) count++;
            }
            return count;
        }

        public boolean isFull() {
            return getBallCount() >= CAROUSEL_CAPACITY;
        }

        public void indexBallClockwise() {
            slots[currentInputSlotIndex] = true;
            // Advance array index 1 position clockwise (modulo 5)
            currentInputSlotIndex = (currentInputSlotIndex + 1) % 5;
        }

        public void resetTracker() {
            for (int i = 0; i < slots.length; i++) {
                slots[i] = false;
            }
            currentInputSlotIndex = 0; // Re-align empty slot to primary input
        }

        public boolean[] getSlotsState() {
            return slots.clone();
        }
    }

    // =========================================================================
    // MODULE 2: DRIVETRAIN & AUTOMATED FEEDER CONTROL
    // =========================================================================

    public class DrivetrainAndFeederControl {
        private final DcMotor frontLeft;
        private final DcMotor frontRight;
        private final DcMotor backLeft;
        private final DcMotor backRight;
        private final DcMotor feederA;
        private final DcMotor feederB;

        private boolean dualInputEnabled = false; // Toggle for single vs dual input logic
        private static final double VELOCITY_THRESHOLD = 0.1; // Min drive threshold for auto-intake

        public DrivetrainAndFeederControl(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor fA, DcMotor fB) {
            this.frontLeft = fl;
            this.frontRight = fr;
            this.backLeft = bl;
            this.backRight = br;
            this.feederA = fA;
            this.feederB = fB;
        }

        public void setDualInputEnabled(boolean enabled) {
            this.dualInputEnabled = enabled;
        }

        public boolean isDualInputEnabled() {
            return dualInputEnabled;
        }

        public void update(Gamepad driverGamepad, VirtualCarouselTracker tracker) {
            // 1. Mecanum Drive Calculation
            double drive  = -driverGamepad.left_stick_y;
            double strafe =  driverGamepad.left_stick_x;
            double turn   =  driverGamepad.right_stick_x;

            double flPower = drive + strafe + turn;
            double blPower = drive - strafe + turn;
            double frPower = drive - strafe - turn;
            double brPower = drive + strafe - turn;

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
            double forwardVelocity = drive; // Velocity direction approximation from joystick input

            if (tracker.isFull()) {
                // Shut off all feeders when capacity (4 balls) is reached
                feederA.setPower(0.0);
                feederB.setPower(0.0);
            } else if (forwardVelocity > VELOCITY_THRESHOLD) {
                // Moving forward toward Input A -> Enable Feeder A, disable Feeder B
                feederA.setPower(1.0);
                feederB.setPower(0.0);
            } else if (forwardVelocity < -VELOCITY_THRESHOLD && dualInputEnabled) {
                // Moving backward toward Input B (if dual input active) -> Enable Feeder B, disable Feeder A
                feederA.setPower(0.0);
                feederB.setPower(1.0);
            } else {
                // Stationary or strafing -> Keep both feeders off to prevent accidental entry/exit
                feederA.setPower(0.0);
                feederB.setPower(0.0);
            }
        }
    }

    // =========================================================================
    // OPMODE EXECUTION & MODULE 3 STATE MACHINE
    // =========================================================================

    // Hardware Declarations
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor feederA, feederB;
    private DcMotor carouselMotor;
    private DigitalChannel sensorInputA, sensorInputB;

    // Constants for Carousel Indexer Motor Encoders
    private static final int TICKS_PER_CAROUSEL_SLOT = 280; // Example encoder ticks for 1/5th rotation (72 deg)
    private static final int TICKS_PER_FULL_360 = 1400;      // 5 slots * 280 ticks = 360 degrees
    private static final double LAUNCH_FLUSH_SPEED = 0.85;

    @Override
    public void runOpMode() {
        // Initialize Hardware Map
        frontLeft     = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight    = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft      = hardwareMap.get(DcMotor.class, "backLeft");
        backRight     = hardwareMap.get(DcMotor.class, "backRight");
        
        feederA       = hardwareMap.get(DcMotor.class, "feederA");
        feederB       = hardwareMap.get(DcMotor.class, "feederB");
        carouselMotor = hardwareMap.get(DcMotor.class, "carouselMotor");

        sensorInputA  = hardwareMap.get(DigitalChannel.class, "sensorInputA");
        sensorInputB  = hardwareMap.get(DigitalChannel.class, "sensorInputB");

        // Motor Direction Setup
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        sensorInputA.setMode(DigitalChannel.Mode.INPUT);
        sensorInputB.setMode(DigitalChannel.Mode.INPUT);

        // Encoder Setup for Carousel
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Instantiations
        VirtualCarouselTracker carouselTracker = new VirtualCarouselTracker();
        DrivetrainAndFeederControl driveControl = new DrivetrainAndFeederControl(
                frontLeft, frontRight, backLeft, backRight, feederA, feederB
        );

        RobotState currentState = RobotState.INTAKE_IDLE;
        int carouselTargetPosition = 0;

        telemetry.addData("Status", "Initialized. Ready for Biobuzz TeleOp.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Drivetrain & Automated Feeder Control
            driveControl.update(gamepad1, carouselTracker);

            // Digital Sensors (Active LOW assumption for optical beam breakers)
            boolean ballDetectedA = !sensorInputA.getState();
            boolean ballDetectedB = !sensorInputB.getState() && driveControl.isDualInputEnabled();
            boolean launchRequested = gamepad2.a || gamepad1.right_bumper; // Operator launch command

            // Module 3: Finite State Machine
            switch (currentState) {

                case INTAKE_IDLE:
                    if (launchRequested) {
                        // Start continuous 360-degree rotation flush past launcher
                        carouselTargetPosition = carouselMotor.getCurrentPosition() + TICKS_PER_FULL_360;
                        carouselMotor.setTargetPosition(carouselTargetPosition);
                        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carouselMotor.setPower(LAUNCH_FLUSH_SPEED);
                        currentState = RobotState.LAUNCHING_ALL;
                    } else if (carouselTracker.isFull()) {
                        currentState = RobotState.CAPACITY_FULL;
                    } else if (ballDetectedA || ballDetectedB) {
                        // Rotate carousel 1 slot clockwise (away from launcher area)
                        carouselTargetPosition = carouselMotor.getCurrentPosition() + TICKS_PER_CAROUSEL_SLOT;
                        carouselMotor.setTargetPosition(carouselTargetPosition);
                        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carouselMotor.setPower(0.5);
                        currentState = RobotState.INDEXING_BALL;
                    }
                    break;

                case INDEXING_BALL:
                    // Wait for carousel motor to finish indexing 1 slot
                    if (!carouselMotor.isBusy()) {
                        carouselTracker.indexBallClockwise();
                        if (carouselTracker.isFull()) {
                            currentState = RobotState.CAPACITY_FULL;
                        } else {
                            currentState = RobotState.INTAKE_IDLE;
                        }
                    }
                    break;

                case CAPACITY_FULL:
                    // Feeders remain OFF automatically via DrivetrainAndFeederControl
                    if (launchRequested) {
                        carouselTargetPosition = carouselMotor.getCurrentPosition() + TICKS_PER_FULL_360;
                        carouselMotor.setTargetPosition(carouselTargetPosition);
                        carouselMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        carouselMotor.setPower(LAUNCH_FLUSH_SPEED);
                        currentState = RobotState.LAUNCHING_ALL;
                    }
                    break;

                case LAUNCHING_ALL:
                    // Wait for continuous 360-degree rotation to complete past passive launcher
                    if (!carouselMotor.isBusy()) {
                        carouselMotor.setPower(0.0);
                        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        carouselTracker.resetTracker();
                        currentState = RobotState.INTAKE_IDLE;
                    }
                    break;
            }

            // Driver Telemetry Feedback
            telemetry.addData("State", currentState);
            telemetry.addData("Ball Count", "%d / %d", carouselTracker.getBallCount(), VirtualCarouselTracker.CAROUSEL_CAPACITY);
            telemetry.addData("Full Capacity?", carouselTracker.isFull() ? "YES (Feeders Locked)" : "No");
            telemetry.update();
        }
    }
}
