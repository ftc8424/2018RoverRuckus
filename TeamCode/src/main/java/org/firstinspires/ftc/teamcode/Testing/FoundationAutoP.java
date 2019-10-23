package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (group = "Auto2019", name = "FoundationAutoP")
public class FoundationAutoP extends LinearOpMode {
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor rightFront;
    private ElapsedTime runtime = new ElapsedTime();
    int step = 0;
    public void MoveInch(double speed, double inches) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition
        double ticks = inches * (560 / (2.95276 * Math.PI));
        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();

        //if the position is less than the number of inches, than it sets the motors to speed
        while (Math.abs(leftBack.getCurrentPosition()) <= ticks) {
            leftBack.setPower(-speed);
            rightBack.setPower(speed);
            leftFront.setPower(-speed);
            rightFront.setPower(speed);
        }
        if (Math.abs(leftBack.getCurrentPosition()) > ticks) {
            step = step + 1;
            runOpMode();
        }
        // leftBack.setPower(speed * 0);
        // rightBack.setPower(speed * 0);
        //leftFront.setPower(speed * 0);
        //rightFront.setPower(speed * 0);

    }
    public void runOpMode() {
    if (step == 0)
    {
        MoveInch(1, 30);
    }
    if(step == 1)
    {
        MoveInch(-1, -30);
    }
    if (step == 2){
        strafe(1, 2);
    }


    }
    public void strafe(double speed, double time) {
        // Ticks is the math for the amount of inches, ticks is paired with getcurrentposition


        //runtime isn't used, this is just a backup call which we don't need


        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //if the position is less than the number of inches, than it sets the motors to speed
        while (runtime.milliseconds() <= time) {
            leftBack.setPower(speed);
            rightBack.setPower(speed);
            leftFront.setPower(speed);
            rightFront.setPower(speed);
        }
        if (runtime.milliseconds() >= time) {
            step = step + 1;
            runOpMode();
        }
        // leftBack.setPower(speed * 0);
        // rightBack.setPower(speed * 0);
        //leftFront.setPower(speed * 0);
        //rightFront.setPower(speed * 0);


}
}


