package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;
import org.firstinspires.ftc.teamcode.OpMode.ClawServo;
import org.firstinspires.ftc.teamcode.OpMode.POV;

@TeleOp(name="Teleop", group ="Teleop")
public class Teleop extends LinearOpMode{

    ApolloHardware robot = new ApolloHardware();
    POV pov = new POV();
    ClawServo claw = new ClawServo();
    @Override
    public void runOpMode()  {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            pov.Drive();
            pov.Turn();
            pov.Slide();
            claw.moveClaw();
        }

    }
}
