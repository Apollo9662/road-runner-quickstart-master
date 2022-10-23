package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;

@TeleOp(name = "Apollo POV", group = "Apollo")
public class ApolloPOV extends LinearOpMode {
    ApolloHardware robot = new ApolloHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            double rightStickX = gamepad1.right_stick_x;

            drive(leftStickX, leftStickY, rightStickX);
        }
    }

    public void drive(double driveSideways, double drive, double turn) {
        double leftFront  = -drive + driveSideways + turn;
        double leftBack = -drive - driveSideways + turn;
        double rightFront = -drive + driveSideways - turn;
        double rightBack = -drive - driveSideways - turn;

        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)),Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1.0)
        {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }

        robot.setLFPower(leftFront);
        robot.setLBPower(leftBack);
        robot.setRFPower(rightFront);
        robot.setRBPower(rightBack);
    }
}
