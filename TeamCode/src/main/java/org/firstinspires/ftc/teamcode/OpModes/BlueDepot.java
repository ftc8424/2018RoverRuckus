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

        do {

        if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
            telemetry.update();
            robot.encoderDrive(this, .25, -.5, -.5, 3);
        }

        else if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
            telemetry.update();
            robot.encoderDrive(this, .25, .5, .5, 3);
        }

        else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
            telemetry.update();
            robot.encoderStrafe(this, .25, .5, 0, 3);
        }

        else if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
            telemetry.update();
            robot.encoderStrafe(this, .25, 0, .5, 3);
        }


        else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
            telemetry.update();
            robot.gyroTurn(this, 0,3);
            robot.encoderDrive(this,.25,-.5,-.5 ,3);
            robot.gyroTurn(this, initialHeading, 3);
        }

        else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
            telemetry.update();
            robot.gyroTurn(this, 0, 3);
            robot.encoderDrive(this,.25, .5, .5, 3);
            robot.gyroTurn(this,initialHeading,3);


        }

        else if (robot.translation.get(1) > 00) {
            telemetry.update();
            robot.gyroTurn(this, 0, 3);
            robot.encoderStrafe(this, .25, .5, 0, 3);
            robot.gyroTurn(this,initialHeading,3);
        }

        else if (robot.translation.get(1) < 00) {
            telemetry.update();
            robot.gyroTurn(this, 0, 3);
            robot.encoderStrafe(this, .25, 0, .5, 3);
            robot.gyroTurn(this,initialHeading,3);
        }

    } while (robot.translation.get(0) != 000 && robot.translation.get(1) != 00);

        super.runDepot(true);
        stopRobot();


    }
}