package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.AutoBase2Vuforia;

@TeleOp
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

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

        }
        vuDriveTo(16, 14, 5);
    }
}
