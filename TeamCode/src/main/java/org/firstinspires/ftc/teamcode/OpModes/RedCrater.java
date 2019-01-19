package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater", group="Red OpMode")
public class RedCrater extends AutoBase {


    public void runOpMode() throws InterruptedException {

        initialHeading = 310;
        deployHeading = 270;
        leftSampleAngle = 275;
        lastHeading = 45;
        lastFinalHeading = 90;
        finalHeading = 180;


        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

        }

        super.runCrater(true);
        stopRobot();


    }
}