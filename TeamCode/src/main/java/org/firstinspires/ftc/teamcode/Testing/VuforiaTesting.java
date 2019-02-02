package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.teamcode.OpModes.AutoBase2Vuforia;

@TeleOp (name = "VuforiaTesting", group = "Concept" )
public class VuforiaTesting extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 130;
        deployHeading = 90;
        leftSampleAngle = 95;
        lastHeading = 225;
        lastFinalHeading = 270;
        finalHeading = 0;

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
            telemetry.addData("Target", robot.targetVisible ? "Visible" : "Invisible");
            telemetry.addData("Heading", robot.getHeading());
            telemetry.update();

        }
        vuDriveTo(16, -14, 100);
        stopRobot();
    }
}
