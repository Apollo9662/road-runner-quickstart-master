package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;



@Autonomous(name = "Apollo Autonomous", group = "Apollo")
public class ApolloAutonomous extends LinearOpMode {
    ApolloHardware robot = new ApolloHardware();

    static final double CENTIMETERS_TO_INCHES = 2.54;

    static final double COUNTS_PER_MOTOR_REV    = (134.4 * 4); // PPR is 134.4; CPR = PPR * 4
    static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double DRIVE_SPEED             = 0.5;
    static final double TURN_SPEED              = 0.3;
    static final double HEADING_THRESHOLD       = 1.0;

    static final double P_TURN_GAIN             = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN            = 0.03;     // Larger is more responsive, but also less stable

    private final double headingOffset = 0;
    private double headingError;
    private double currentHeading = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.log().add("> Init done");
        telemetry.log().add("> Ready to start");

        waitForStart();

        drive(DRIVE_SPEED, 50 * CENTIMETERS_TO_INCHES, currentHeading);
        turnToHeading(TURN_SPEED, 90);
        drive(DRIVE_SPEED, 50 * CENTIMETERS_TO_INCHES, currentHeading);
    }

    public void drive(double speed, double inches, double heading) {
        int leftTarget;
        int rightTarget;

        if (opModeIsActive()) {
            int moveCount = (int)(inches * COUNTS_PER_INCH);
            leftTarget = robot.LeftFront.getCurrentPosition() + moveCount;
            rightTarget = robot.RightFront.getCurrentPosition() - moveCount;

            robot.setDriveTarget(leftTarget, rightTarget);

            robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            double maxDriveSpeed = Math.abs(speed);
            robot.driveRobot(maxDriveSpeed, 0);

            while (opModeIsActive() && robot.amountOfMotorsBusy()) {
                double turnSpeed = -getSteeringCorrection(heading, P_DRIVE_GAIN);

                if (inches < 0) {
                    turnSpeed *= -1.0;
                }
//test
                Log.d(robot.TAG, "turn speed: " + turnSpeed);
                robot.driveRobot(maxDriveSpeed, turnSpeed);
            }

            robot.setDrivePower(0, 0);

            robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(100);
        }
    }

    public void turnToHeading(double speed, double heading) {
        currentHeading = heading;

        getSteeringCorrection(heading, P_DRIVE_GAIN);

        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            double turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            turnSpeed = Range.clip(turnSpeed, -speed, speed);

            Log.d(robot.TAG, "turn speed: " + turnSpeed);
            robot.driveRobot(0, turnSpeed);
        }

        robot.setDrivePower(0, 0);
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        // Get the robot heading by applying an offset to the IMU heading
        double robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = desiredHeading - robotHeading;
        Log.d(robot.TAG, "hError: " + headingError + ", DesiredHeading: " + desiredHeading);

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;


        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getRawHeading() {
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        Log.d(robot.TAG, "heading: " + angles.firstAngle);
        return angles.firstAngle;
    }
}
