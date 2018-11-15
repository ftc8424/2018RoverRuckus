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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        super.runDepot();


    }
}