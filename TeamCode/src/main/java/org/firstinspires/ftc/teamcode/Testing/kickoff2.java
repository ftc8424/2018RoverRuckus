package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "race", group = "kickoff2")
public class kickoff2 extends OpMode {
    DcMotor lb;
    DcMotor rb;
    DcMotor lf;
    DcMotor rf;
    ElapsedTime time = new ElapsedTime();
    public void init() {
        lb = hardwareMap.dcMotor.get("Left Back");
        rb = hardwareMap.dcMotor.get("Right Back");
        lf = hardwareMap.dcMotor.get("Left Front");
        rf = hardwareMap.dcMotor.get("Right Front");
    }

    public void loop() {
        if (Math.abs(gamepad1.left_stick_y) > .05) {
            lb.setPower(gamepad1.left_stick_y);
            rb.setPower(-gamepad1.left_stick_y);
            lf.setPower((gamepad1.left_stick_y));
            rf.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Moving", gamepad1.left_stick_y);
        }
        if (Math.abs(gamepad1.left_stick_x) > .05) {
            lf.setPower(-gamepad1.left_stick_x);
            lb.setPower(gamepad1.left_stick_x);
            rf.setPower(-gamepad1.left_stick_x);
            rb.setPower(gamepad1.left_stick_x);
            telemetry.addData("Strafing", gamepad1.left_stick_x);

        }
        if (Math.abs(gamepad1.right_stick_x) > .05) {
            lb.setPower(gamepad1.right_stick_x);
            lf.setPower(gamepad1.right_stick_x);
            rb.setPower(gamepad1.right_stick_x);
            rf.setPower(gamepad1.right_stick_x);
            telemetry.addData("Turning", gamepad1.right_stick_x);

        }
    }

    public void stop() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

        }
}

