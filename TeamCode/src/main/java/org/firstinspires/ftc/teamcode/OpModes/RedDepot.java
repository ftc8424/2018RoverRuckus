package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

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
        boolean targetVisible = false;

        while (!isStopRequested() && !isStarted()) {
            telemetry.addData("Gyro Status", robot.imu.isGyroCalibrated() ? "Calibrated - Ready for Start" : "Calibrating - DO NOT START");

            if (robot.VuforiaTorch()){
                robot.camera.setFlashTorchMode(false);
            }
            telemetry.update();

        }

        do {

        if (robot.translation.get(0) > 000 && robot.translation.get(1) > 00) {
            telemetry.update();
            robot.encoderDrive(this, .25, -.5, -.5, 3);
        }

        else if (robot.translation.get(0) < 000 && robot.translation.get(1) < 00) {
            telemetry.update();
            robot.encoderDrive(this, .25, .5, .5, 3);
        }

        else if (robot.translation.get(0) > 000 && robot.translation.get(1) < 00) {
            telemetry.update();
            robot.encoderStrafe(this, .25, .5, 0, 3);
        }

        else if (robot.translation.get(0) < 000 && robot.translation.get(1) > 00) {
            telemetry.update();
            robot.encoderStrafe(this, .25, 0, .5, 3);
        }


        else if (robot.translation.get(0) > 000 && robot.translation.get(1) == 00) {
            telemetry.update();
            robot.gyroTurn(this, 180,3);
            robot.encoderDrive(this,.25,-.5,-.5 ,3);
            robot.gyroTurn(this, initialHeading, 3);
        }

        else if (robot.translation.get(0) < 000 && robot.translation.get(1) == 00) {
            telemetry.update();
            robot.gyroTurn(this, 180, 3);
            robot.encoderDrive(this,.25, .5, .5, 3);
            robot.gyroTurn(this,initialHeading,3);


        }

        else if (robot.translation.get(1) < 00) {
            telemetry.update();
            robot.gyroTurn(this, 180, 3);
            robot.encoderStrafe(this, .25, .5, 0, 3);
            robot.gyroTurn(this,initialHeading,3);
        }

        else if (robot.translation.get(1) > 00) {
            telemetry.update();
            robot.gyroTurn(this, 180, 3);
            robot.encoderStrafe(this, .25, 0, .5, 3);
            robot.gyroTurn(this,initialHeading,3);
        }
    } while (robot.translation.get(0) != 000 && robot.translation.get(1) != 00);


        super.runDepot(true);
        stopRobot();


    }
}
