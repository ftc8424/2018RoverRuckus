package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Depot Unlatched", group="Blue OpMode")
public class BlueDepotUnlatched extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 40;
        deployHeading = 90;
        halfHeading = 180;
        finalHeading = 0;
        lastHeading = 145;
        lastFinalHeading = 155;
        leftSampleAngle = 0;
        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
            telemetry.update();
        }
        super.runDepot(false);
        stopRobot();


    }
}