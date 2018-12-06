package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Depot", group="Blue OpMode")
public class BlueDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 40;
        deployHeading = 90;
        halfHeading = 180;
        finalHeading = 0;
        lastHeading = 145;
        lastFinalHeading = 155;
        robot.robot_init(hardwareMap,true);
        initRobot();

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");
        }
        super.runDepot();


    }
}