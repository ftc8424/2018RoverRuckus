package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Crater Double Sample", group="Red OpMode")
public class RedCraterDoubleSample extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 310;
        deployHeading = 270;
        zeroHeading = 180;
        lastHeading = 45;
        lastFinalHeading = 90;

        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

        }


        super.runCrater(true, true);
        stopRobot();



    }
}