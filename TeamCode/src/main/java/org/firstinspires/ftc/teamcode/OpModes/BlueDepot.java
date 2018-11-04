package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Depot", group="Blue OpMode")
public class BlueDepot extends AutoBase {


    public void runOpMode() throws InterruptedException {
        initialHeading = 40;
        finalHeading = 0;
        robot.robot_init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runDepot();


    }
}