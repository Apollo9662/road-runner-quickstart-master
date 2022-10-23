/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpMode;

import static com.sun.tools.doclint.Entity.and;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.ApolloHardware;


public class POV extends LinearOpMode {

    ApolloHardware robot = new ApolloHardware();

    double power;
    double turn;
    double slide;


    @Override
    public void runOpMode() {
        //robot.init(this);

        Log.d(robot.TAG, "> Robot is ready");
        waitForStart();

        while (opModeIsActive()) {

        Drive();
        Turn();
        Slide();

        }
    }

    public void Drive() {
        power = gamepad1.right_stick_y;
        robot.LeftBack.setPower(power);
        robot.LeftFront.setPower(power);
        robot.RightBack.setPower(power);
        robot.RightFront.setPower(power);
    }

    public void Turn(){
        turn = gamepad1.left_stick_x;

        if(turn > 0 ) {
            robot.RightFront.setPower(turn);
            robot.RightBack.setPower(turn);
            robot.LeftFront.setPower(-turn);
            robot.LeftBack.setPower(-turn);
        }

        else if(turn < 0){
            robot.RightFront.setPower(-turn);
            robot.RightBack.setPower(-turn);
            robot.LeftFront.setPower(turn);
            robot.LeftBack.setPower(turn);
        }

        else {
            robot.RightFront.setPower(0);
            robot.RightBack.setPower(0);
            robot.LeftFront.setPower(0);
            robot.LeftBack.setPower(0);
        }
    }

    public void Slide(){
        slide = gamepad1.right_stick_x;

        if(slide > 0){
            robot.RightFront.setPower(-slide);
            robot.RightBack.setPower(slide);
            robot.LeftFront.setPower(slide);
            robot.LeftBack.setPower(-slide);

        }

        else if(slide < 0){
            robot.RightFront.setPower(slide);
            robot.RightBack.setPower(-slide);
            robot.LeftFront.setPower(-slide);
            robot.LeftBack.setPower(slide);
        }
        else{
            robot.RightFront.setPower(0);
            robot.RightBack.setPower(0);
            robot.LeftFront.setPower(0);
            robot.LeftBack.setPower(0);
        }


    }


}




