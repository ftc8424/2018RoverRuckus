package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Depot", group="Red OpMode")
public class RedDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 225;
        deployHeading = 270;
        halfHeading = 0;
        leftSampleAngle = 180;
        lastHeading = 45;
        lastFinalHeading = 90;
        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            telemetry.update();

        }
        super.runDepot(true);
        stopRobot();


    }
}
