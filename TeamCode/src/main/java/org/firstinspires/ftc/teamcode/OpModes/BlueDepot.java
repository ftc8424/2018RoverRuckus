package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Depot", group="Blue OpMode")
public class BlueDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 45;
        deployHeading = 90;
        halfHeading = 180;
        leftSampleAngle = 0;
        lastHeading = 225;
        lastFinalHeading = 270;

        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();
        }

        super.runDepot(true);
        stopRobot();


    }
}