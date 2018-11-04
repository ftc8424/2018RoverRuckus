package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Depot", group="Blue OpMode")
public class BlueDepot extends AutoDepot {


    public void runOpMode() throws InterruptedException {
        initialHeading = 40;
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.reallyRunOpMode();


    }
}