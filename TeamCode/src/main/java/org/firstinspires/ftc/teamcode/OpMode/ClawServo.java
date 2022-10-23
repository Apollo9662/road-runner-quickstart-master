package org.firstinspires.ftc.teamcode.OpMode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;

@TeleOp(name = "Claw Test", group = "Apollo")
public class ClawServo extends LinearOpMode{

    ApolloHardware robot = new ApolloHardware();

    double ServoCurrentPosition = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Log.d(robot.TAG, "> " + "Robot init completed");

        Log.d(robot.TAG, "> " + "Robot is ready");
        waitForStart();

        while (opModeIsActive()) {
            moveClaw();
        }
    }

    public void moveClaw(){ // Claw Range is between 0.13 and 0.275

        if (gamepad1.right_trigger > 0) {
            ServoCurrentPosition = 0.275; // Open
        } else {
            ServoCurrentPosition = 0.13;
        }

        robot.Claw.setPosition(ServoCurrentPosition);
        Log.d(robot.TAG, "Claw Servo Position is: " + ServoCurrentPosition);
    }
}
