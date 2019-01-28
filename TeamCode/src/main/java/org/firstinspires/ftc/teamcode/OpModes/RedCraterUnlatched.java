package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater Unlatched", group="Red OpMode")
public class RedCraterUnlatched extends AutoBase2Vuforia {


    public void runOpMode() throws InterruptedException {
        initialHeading = 310;
        deployHeading = 270;
        leftSampleAngle = 275;
        lastHeading = 45;
        lastFinalHeading = 90;
        finalHeading = 180;

        robot.robot_init(hardwareMap,true);
        initRobot();

        robot.targetsRoverRuckus.activate();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            robot.vuforiaTesting(this);
            telemetry.update();



        }

        super.runCrater(false, false);

        stopRobot();
    }
}