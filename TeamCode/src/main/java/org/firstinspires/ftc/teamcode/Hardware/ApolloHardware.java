/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Hardware;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.R;

import java.util.Map;

public class ApolloHardware {

    public String TAG = "ApolloHardware";

    public DcMotor LeftFront, LeftBack, RightFront, RightBack = null;

    public DcMotor Elevator = null;

    private final DcMotor.RunMode motorsRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    private final DcMotor.RunMode elevatorRunMode = DcMotor.RunMode.RUN_USING_ENCODER;

    public BNO055IMU imu;

    public Servo Claw = null;
    public double CLAW_OPEN_POSITION = 0.13;
    public double CLAW_CLOSED_POSITION = 0.275;

    public void init(HardwareMap hMap) {
        LeftFront = hMap.get(DcMotor.class, "left_front");
        LeftBack = hMap.get(DcMotor.class, "left_back");
        RightFront = hMap.get(DcMotor.class, "right_front");
        RightBack = hMap.get(DcMotor.class, "right_back");

        // Elevator = hMap.get(DcMotor.class, "elevator");

        // resetElevatorEncoder();
        resetDrivingEncoders();

        LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Elevator.setDirection(DcMotorSimple.Direction.FORWARD);

        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightFront.setPower(0);
        RightBack.setPower(0);

        // Elevator.setPower(0);

        Claw = hMap.get(Servo.class, "claw");

        Claw.setPosition(CLAW_OPEN_POSITION);

        imu = hMap.get(BNO055IMU.class, "imu1");
        if (!initImu()) {
            Log.d(TAG, " fail to init imu1");
            imu =  hMap.get(BNO055IMU.class, "imu2");
            if (!initImu()) {
                Log.d(TAG, " fail to init imu2");
            }
        }

        Log.d(TAG, " > Hardware Initialized");
    }

    public void resetDrivingEncoders() {
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront.setMode(motorsRunMode);
        LeftBack.setMode(motorsRunMode);
        RightFront.setMode(motorsRunMode);
        RightBack.setMode(motorsRunMode);
    }

    public void resetElevatorEncoder() {
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Elevator.setMode(elevatorRunMode);
    }

    public void setRunMode(DcMotor.RunMode runMode) {
        LeftFront.setMode(runMode);
        LeftBack.setMode(runMode);
        RightFront.setMode(runMode);
        RightBack.setMode(runMode);
    }

    public int returnLFEncoder() {
        return LeftFront.getCurrentPosition();
    }
    public int returnLBEncoder() {
        return LeftBack.getCurrentPosition();
    }
    public int returnRFEncoder() {
        return -RightFront.getCurrentPosition();
    }
    public int returnRBEncoder() {
        return -RightBack.getCurrentPosition();
    }

    public void setDriveTarget(int leftPosition, int rightPosition) {
        LeftFront.setTargetPosition(leftPosition);
        LeftBack.setTargetPosition(leftPosition);
        RightFront.setTargetPosition(rightPosition);
        RightBack.setTargetPosition(rightPosition);
    }

    public void setDrivePower(double leftDrivePower, double rightDrivePower) {
        LeftFront.setPower(leftDrivePower);
        LeftBack.setPower(leftDrivePower);
        RightFront.setPower(rightDrivePower);
        RightBack.setPower(rightDrivePower);
    }

    public void driveRobot(double drive, double turn) {
        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        Log.d(TAG, "left power: " + leftSpeed + " right power: " + rightSpeed);
        setDrivePower(leftSpeed, rightSpeed);
    }

    public boolean amountOfMotorsBusy() {
        boolean isBusy = LeftFront.isBusy() && RightFront.isBusy();

        //if (LeftFront.isBusy()) motorsBusy++;
        //if (LeftBack.isBusy()) motorsBusy++;
        //if (RightFront.isBusy()) motorsBusy++;
        // if (RightBack.isBusy()) motorsBusy++;

        return isBusy;
    }

    public void moveSideways(double power) {
        setLFPower(power);
        setLBPower(-power);
        setRFPower(power);
        setRBPower(-power);
    }

    public void setLFPower(double power) {
        LeftFront.setPower(power);
    }

    public void setLBPower(double power) {
        LeftBack.setPower(power);
    }

    public void setRFPower(double power) {
        RightFront.setPower(power);
    }

    public void setRBPower(double power) {
        RightBack.setPower(power);
    }

    public void setLFDirection(DcMotor.Direction direction) {
        LeftFront.setDirection(direction);
    }

    public void setLBDirection(DcMotor.Direction direction) {
        LeftBack.setDirection(direction);
    }

    public void setRFDirection(DcMotor.Direction direction) {
        RightFront.setDirection(direction);
    }

    public void setRBDirection(DcMotor.Direction direction) {
        RightBack.setDirection(direction);
    }

    public void setClawPosition(double position) {
        Claw.setPosition(position);
    }

    public void switchClawState() {
        if (Claw.getPosition() > CLAW_OPEN_POSITION) {
            setClawPosition(CLAW_CLOSED_POSITION);
        } else {
            setClawPosition(CLAW_OPEN_POSITION);
        }
    }

    public boolean initImu() {
        boolean success;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        success = imu.initialize(parameters);

        Log.d(TAG, "init imu: " + success);
        return success;
    }

    public double robotAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return(AngleUnit.DEGREES.fromUnit(angles.angleUnit,angles.firstAngle));
    }

    public double getRealRobotAngle(double angle){
        return -(robotAngle() - angle);
    }

    public double angleWrap(double angle) {
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }
}

