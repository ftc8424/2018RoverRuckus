package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.Hardware.AMLChampionshipRobot;
@Autonomous(name="Blue Taxi", group="Blue OpMode")

public class BlueTaxi extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    protected AMLChampionshipRobot robot = new AMLChampionshipRobot();
    protected double LiftUp = 300;
    protected double LiftDown = 0;

    public void initRobot() {
        robot.initMotor(true);
        robot.initServo();
        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        robot.imu.initialize(parameters);
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.initVuforia();

    }
    @Override
    public void runOpMode() throws InterruptedException {

        robot.robot_init(hardwareMap,true);
        initRobot();

        robot.targetsRoverRuckus.activate();

        robot.camera = CameraDevice.getInstance();
        boolean acquired = false;
        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.imu.isGyroCalibrated()) {
                if (robot.VuforiaTorch()) {
                    acquired = true;
                } else if (!acquired) {
                    robot.camera.setFlashTorchMode(true); // Turn on to alert setup to acquire
                }
            }
            if (acquired) {
                robot.camera.setFlashTorchMode(false); // Turn off to alert setup is acquired
            }
            robot.vuforiaTesting(this);
            telemetry.addData("Heading", robot.getHeading());
            telemetry.update();
        }
        
        switch (one ) {

            case one:

                robot.encoderDrive(this, .5, -27, -27, 4);


                robot.encoderDrive(this, .5, 55, 55, 4);
                robot.gyroTurn(this, 270, 3);

                robot.encoderDrive(this, .5, -65, -65, 5);

                break;


            case two:
                robot.encoderDrive(this, .5, -30, -30, 4);


                robot.gyroTurn(this, 90, 4);

                robot.encoderDrive(this, .75, 50, 50, 5);
                robot.gyroTurn(this, 0, 3);

                robot.encoderDrive(this, .5, 50, 50, 5);


                break;

            case notSense:
                robot.encoderDrive(this, .5, -30, -30, 4);


                robot.gyroTurn(this, 270, 3);
                robot.encoderDrive(this, .5, 70, 70, 4);
                robot.gyroTurn(this, 0, 3);

                robot.encoderDrive(this, .5, -45, -45, 5);
                robot.LiftMotor.setPower(0);


                break;
        }


        }


    private static final int notSense = -1;
    private static final int one = 1;
    private static final int two = 2;
    private static  boolean sensed = false;
    private static int ferryPos = 0;
    private static int times = 0;

    private int ferryTarget() {

        do {
            if (robot.targetsRoverRuckus.getName() == "Front-Craters") {
                sensed = true;
                ferryPos = one;
            } else if (robot.targetsRoverRuckus.getName() == "Red-Footprint") {
                sensed = true;
                ferryPos = one;
            } else if (robot.targetsRoverRuckus.getName() == "Back-Space") {
                sensed = true;
                ferryPos = two;
            } else if (robot.targetsRoverRuckus.getName() == "Blue-Rover") {
                sensed = true;
                ferryPos = two;
            }
            times++;
            sleep(250);
        } while (opModeIsActive() && sensed == false && times <= 4);
        if (times > 4 && sensed == false) {
            ferryPos = notSense;
        }
        return ferryPos;
    }
}

