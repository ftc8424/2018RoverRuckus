package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Blue Crater Unlatched", group="Blue OpMode")
public class BlueCraterUnlatched extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 130;
        deployHeading = 90;
        halfHeading = 180;
        lastHeading = 225;
        lastFinalHeading = 270;

        robot.robot_init(hardwareMap,true);
        initRobot();
        robot.targetsRoverRuckus.activate();

        robot.camera = CameraDevice.getInstance();
        boolean acquired = false;
        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if ( robot.imu.isGyroCalibrated() ) {
                if (robot.VuforiaTorch()) {
                    acquired = true;
                } else if ( !acquired ) {
                    robot.camera.setFlashTorchMode(true); // Turn on to alert setup to acquire
                }
            }
            if ( acquired ) {
                robot.camera.setFlashTorchMode(false); // Turn off to alert setup is acquired
            }
            robot.vuforiaTesting(this);
            telemetry.update();

        }

        super.runCrater(false);
        stopRobot();


    }
}