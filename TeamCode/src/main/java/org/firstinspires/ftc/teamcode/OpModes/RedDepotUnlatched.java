package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            robot.vuforiaTesting(this);
            telemetry.update();

        }
        super.runDepot(false);
        stopRobot();


    }
}
