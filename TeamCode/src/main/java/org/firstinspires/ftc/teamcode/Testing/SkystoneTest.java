package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group= "skystone2019",name ="Auto2019" )
public class skystoneAuto2019 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;




    public void runOpMode() {
        // lb.setPower();
        waitforStart();
        robot.gyroCalibrate();
        if (leftFront.getCurrentPosition() < 1846) {
            leftBack.setPower(1);
            rightBack.setPower(1);
            leftFront.setPower(1);
            rightFront.setPower(1);
        } else {
            leftBack.setPower(0);
            rightBack.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
        }

    }
}