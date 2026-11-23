package org.firstinspires.ftc.team19938;

import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous Program", group = "Autonomous")
public class BioBuzzAuto19938 extends LinearOpMode {
    public static final double BALL_THRESHOLD = 8;
    public static final int TICKS_PER_FULL_360 = 288;
    public static final double INDEXER_SPEED = 0.85;
    public static final double FLUSH_SPEED = 0.2;

    public static final double DECELERATION_SPEED = 0.45;
    public static final int TICKS_PER_SPINDEXER_SLOT = 58;
    public static final double LAUNCH_SPEED = 2300;
    public static final int TICK_THRESHOLD = 8;
    public static final int DETECTION_TIMMEOUT_MS = 250;


    // This declares the four motors needed
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor feederRight;
    private DcMotor feederLeft;
    private DcMotorEx launcher;
    private DcMotorEx spindexer;
    private DistanceSensor distanceSensorRight;
    private DistanceSensor distanceSensorLeft;
    private SpindexerIndexer spindexerIndexer;
    private DirectionalIntaking directionalIntaking;
    private SpindexerState state;
    private int spindexerPos;
    private Timer driveTimer = null;


    public enum SpindexerState {
       Startup,
        Launch,
        Move,
        Stop,

    }

    IMU imu;

    @Override

    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right");

        feederRight = hardwareMap.get(DcMotor.class, "frontFeed");
        feederLeft = hardwareMap.get(DcMotor.class, "backFeed");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer");

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
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setTargetPosition(0);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.setVelocityPIDFCoefficients(5.46, 0.55, 0.0, 54.6);
        spindexer.setPositionPIDFCoefficients(5.0);
        spindexer.setPower(INDEXER_SPEED);

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
                feederLeft,
                imu
        );

        spindexerPos = 0;

        waitForStart();
        launcher.setVelocity(LAUNCH_SPEED);

        state = SpindexerState.Startup;

        while (opModeIsActive()) {


            switch (state){
                case Startup:
                    if(driveTimer == null){
                        driveTimer = new Timer();
                        driveTimer.reset();
                    } else if(driveTimer.get(TimeUnit.MILLISECONDS) > 11000){
                        driveTimer = null;
                        spindexerPos -= TICKS_PER_FULL_360;
                        spindexer.setTargetPosition(spindexerPos);
                        spindexer.setPower(FLUSH_SPEED);
                        state = SpindexerState.Launch;
                    }
                    break;
                case Launch:
                    if(isAtTargetPosition()){
                        driveTimer = new Timer();
                        driveTimer.reset();
                        state = SpindexerState.Move;
                    }
                    break;
                case Move:
                    if(driveTimer.get(TimeUnit.MILLISECONDS) < 500){
                        drive(-0.5, 0, 0);
                    } else {
                        driveTimer = null;
                        state = SpindexerState.Stop;
                    }
                    break;
                case Stop:
                    drive(0, 0, 0);
                    break;
            }


        }
    }
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;  // make this slower for outreaches

        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
    private boolean isAtTargetPosition() {
        int remainingTicks = Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition());
        return remainingTicks <= TICK_THRESHOLD;
    }

}





