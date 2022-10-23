package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;

// test git

@TeleOp(name="Unit Test", group="Apollo")
public class UnitTest extends LinearOpMode {

    ApolloHardware robot = new ApolloHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        Log.d(robot.TAG, "> " + "Robot ready to start");
        waitForStart();

        Log.d(robot.TAG, "> " + "Robot started");

        double power;

        DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction leftBackDirection = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rightBackDirection = DcMotorSimple.Direction.FORWARD;

        while (opModeIsActive()) {
            power = gamepad1.left_stick_y;

            if (gamepad1.right_trigger == 0) {
                robot.setClawPosition(0);
            }
            if (gamepad1.right_trigger < 0.8 && gamepad1.right_trigger > 0) {
                robot.setClawPosition(0.5);
            }
            if (gamepad1.right_trigger > 0.8) {
                robot.setClawPosition(1);
            }
            if (gamepad1.right_stick_button) {
                robot.switchClawState();
            }

            if (gamepad1.left_stick_button) {
                robot.resetDrivingEncoders();
            }


            if (gamepad1.left_bumper) robot.moveSideways(1);
            else robot.moveSideways(0);

            if (gamepad1.y) {
                robot.setLFPower(power);
                Log.d(robot.TAG, "> " + "Turned on left front with power: " + power);
            }

            if (gamepad1.x) {
                robot.setLBPower(power);
                Log.d(robot.TAG, "> " + "Turned on left back with power: " + power);
            }

            if (gamepad1.b) {
                robot.setRFPower(power);
                Log.d(robot.TAG, "> " + "Turned on right front with power: " + power);
            }

            if (gamepad1.a) {
                robot.setRBPower(power);
                Log.d(robot.TAG, "> " + "Turned on right back with power: " + power);
            }

            if (power != 0) {
                Log.d(robot.TAG, "> " + "Left front encoder: " + robot.returnLFEncoder());
                Log.d(robot.TAG, "> " + "Left back encoder: " + robot.returnLBEncoder());
                Log.d(robot.TAG, "> " + "Right front encoder: " + robot.returnRFEncoder());
                Log.d(robot.TAG, "> " + "Right back encoder: " + robot.returnRBEncoder());

                telemetry.addLine("> " + "Left front encoder: " + robot.returnLFEncoder());
                telemetry.addLine("> " + "Left back encoder: " + robot.returnLBEncoder());
                telemetry.addLine("> " + "Right front encoder: " + robot.returnRFEncoder());
                telemetry.addLine("> " + "Right back encoder: " + robot.returnRBEncoder());
                telemetry.update();
            }

            if (gamepad1.dpad_up) {
                leftFrontDirection = leftFrontDirection.inverted();
                robot.setLFDirection(leftFrontDirection);
                Log.d(robot.TAG,"> " + "Inverted left front");
            }
            if (gamepad1.dpad_left) {
                leftBackDirection = leftBackDirection.inverted();
                robot.setLBDirection(leftBackDirection);
                Log.d(robot.TAG,"> " + "Inverted left back");
            }
            if (gamepad1.dpad_right) {
                rightFrontDirection = rightFrontDirection.inverted();
                robot.setRFDirection(rightFrontDirection);
                Log.d(robot.TAG,"> " + "Inverted right front");
            }
            if (gamepad1.dpad_down) {
                rightBackDirection = rightBackDirection.inverted();
                robot.setRBDirection(rightBackDirection);
                Log.d(robot.TAG,"> " + "Inverted right back");
            }
        }
    }
}
