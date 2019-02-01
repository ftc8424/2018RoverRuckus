package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Blue Depot Unlatched", group="Blue OpMode")
public class BlueDepotUnlatched extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 40;
        deployHeading = 90;
        halfHeading = 180;
        finalHeading = 0;
        lastHeading = 145;
        lastFinalHeading = 155;
        leftSampleAngle = 0;
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
        super.runDepot(false);
        stopRobot();


    }
}