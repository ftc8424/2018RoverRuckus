package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Hardware.AMLChampionshipRobot;
import org.firstinspires.ftc.teamcode.Hardware.Constants;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;

@Autonomous(name="VoforiaTaxiTest", group="Blue OpMode")

public class VuforiaTesting extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    protected AMLChampionshipRobot robot = new AMLChampionshipRobot();
    protected double LiftUp = 300;
    protected double LiftDown = 0;
    public List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    public VuforiaLocalizer vuforia;

    /*
     * Here is the Vuforia-specific settings, only using Vuforia in Championship.
     */
    public VuforiaTrackables targetsRoverRuckus;
    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    public static final float mmPerInch        = 25.4f;
    public static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    public static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;





    public void initRobot() {
        robot.initMotor(true);
        robot.initServo();
        robot.setEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.LiftMotor.setPower(0);

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
            for (VuforiaTrackable trackable : robot.allTrackables) {
                if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    ferryPosition();
                    telemetry.addData("FerryPosition", ferryPos);
                    robot.targetVisible = true;
                }
                telemetry.update();
            }
        }

        for (VuforiaTrackable trackable : robot.allTrackables) {
                    if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        telemetry.addData("Visible Target", trackable.getName());
                ferryPosition();
                telemetry.addData("FerryPosition", ferryPos);
                robot.targetVisible = true;
            }
            ferryPosition();
            telemetry.addData("FerryPosition", ferryPos);

            sleep(5000);
        }
    }

        /*switch (ferryTarget()) {

            case one:
                telemetry.addData("VuMark", robot.targetsRoverRuckus.getName());
                telemetry.addData("FerryPosition", ferryPos);
                sleep(3000);

            case two:
                telemetry.addData("VuMark", robot.targetsRoverRuckus.getName());
                telemetry.addData("FerryPosition", ferryPos);
                sleep(3000);


            case notSense:
                telemetry.addData("VuMark", robot.targetsRoverRuckus.getName());
                telemetry.addData("FerryPosition", ferryPos);
                sleep(3000);

        }*/



    private static int notSense = -1;
    private static final int one = 1;
    private static final int two = 2;
    private static boolean sensed = false;
    private static int ferryPos = 0;
    private static int times = 0;

    public int ferryPosition() {
        for (VuforiaTrackable trackable : robot.allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {

                do {
                    if (trackable.getName() == "Front-Craters") {
                        sensed = true;
                        ferryPos = one;
                    } else if (trackable.getName() == "Red-Footprint") {
                        sensed = true;
                        ferryPos = one;
                    } else if (trackable.getName() == "Back-Space") {
                        sensed = true;
                        ferryPos = two;
                    } else if (trackable.getName() == "Blue-Rover") {
                        sensed = true;
                        ferryPos = two;
                    }
                    times++;
                    sleep(250);
                } while (opModeIsActive() && sensed == false && times <= 4);
                if (times > 4 && sensed == false) {
                    ferryPos = notSense;
                }
            }
        }
        return ferryPos;
    }



}
