package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Crater Unlatched", group="Blue OpMode")
public class BlueCraterUnlatched extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 130;
        deployHeading = 90;
        halfHeading = 180;
        lastHeading = 225;
        lastFinalHeading = 270;

        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            telemetry.update();

        }
        super.runCrater(false);


    }
}