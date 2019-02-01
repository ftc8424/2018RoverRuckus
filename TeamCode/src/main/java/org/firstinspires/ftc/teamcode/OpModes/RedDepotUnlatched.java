package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Red Depot Unlatched", group="Red OpMode")
public class RedDepotUnlatched extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 220;
        deployHeading = 270;
        finalHeading = 180;
        halfHeading = 0;
        lastHeading = 325;
        lastFinalHeading = 335;
        leftSampleAngle = 180;
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
            telemetry.addData("Heading", robot.getHeading());
            telemetry.update();

        }
        super.runDepot(false);
        stopRobot();


    }
}
