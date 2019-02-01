package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.CameraDevice;

@Autonomous(name="Red Crater Double Sample", group="Red OpMode")
public class RedCraterDoubleSample extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 310;
        deployHeading = 270;
        zeroHeading = 180;
        lastHeading = 45;
        lastFinalHeading = 90;

        robot.robot_init(hardwareMap,true);
        initRobot();

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


        super.runCrater(true, true);
        stopRobot();



    }
}