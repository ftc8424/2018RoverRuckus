package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Crater", group="Blue OpMode")
public class BlueCrater extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 130;
        deployHeading = 90;
        zeroHeading = 0;
        lastHeading = 225;
        lastFinalHeading = 270;

        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            telemetry.update();

        }
        super.runCrater(true);


    }
}