package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="AutonomousMode", group="Linear Opmode")

public class AutoOpModev1 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Servo clawServo1 = null;
    private Servo clawServo2 = null;
    private Servo tailServo = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor forkliftDrive = null;
    private ColorSensor color_sensor = null;
    ElapsedTime eTime = new ElapsedTime();


    // All variables needed for Servos
    private boolean servoDirection = true;
    private boolean buttonPress = true;
    private double servoPos = 0;
    private boolean moveBackOrFront = true;
    private boolean UpOrDown = true;
    private int loop = 0;
    private boolean moveUpOrDown = true;
    int InsideLoopUp = 0;
    int InsideLoopDown = 0;
    double speed;
    double math = Math.random() * ( 1 - 0 );
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        clawServo1 = hardwareMap.get(Servo.class, "clawServo1");
        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");
        tailServo = hardwareMap.get(Servo.class, "tailServo");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        forkliftDrive = hardwareMap.get(DcMotor.class, "forklift_drive");
        color_sensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*

        //Move back towards the jewls
        eTime.reset();
        while(eTime.time() <  0.25) {
            leftDrive.setPower(-0.25);
            rightDrive.setPower(-0.25);
        }
        //stops

        leftDrive.setPower(0.0);
        rightDrive.setPower(0.0);
        */


        //Move tail down
        tailServo.setDirection(Servo.Direction.FORWARD);
        tailServo.setPosition(0.75);
        wait(400);

        // COLOR SENSOR PART
        //IF COLOR SENSOR EQUALS BLAH COLOR THEN MOVE THAT WAY

        if(color_sensor.alpha() > 4){
            eTime.reset();
            while (eTime.time() < 0.25) {
                leftDrive.setPower(-0.35);
                rightDrive.setPower(0.35);
            }
            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        } else  {
            //turn right THIS IS A PLACEHOLDER
            eTime.reset();
            while (eTime.time() < 0.25) {
                leftDrive.setPower(0.35);
                rightDrive.setPower(-0.35);
            }

            leftDrive.setPower(0.0);
            rightDrive.setPower(0.0);
        }
        //move tail back up
        tailServo.setDirection(Servo.Direction.REVERSE);
        tailServo.setPosition(0.25);
        wait(400);

        //Turn back to straight position
        eTime.reset();
        while(eTime.time() <  0.25) {
            leftDrive.setPower(-0.35);
            rightDrive.setPower(0.35);
        }
        //THIS is a comment




        //move forward off the balance
        eTime.reset();
        while(eTime.time() <  2) {
            leftDrive.setPower(0.35);
            rightDrive.setPower(0.35);
        }


        //Turn once off the ramp
        eTime.reset();
        while(eTime.time() <  0.65) {
            leftDrive.setPower(-0.25);
            rightDrive.setPower(0.25);
        }

        //move forward into the safezone
        eTime.reset();
        while(eTime.time() <  2) {
            leftDrive.setPower(0.35);
            rightDrive.setPower(0.35);
        }


            telemetry.addData("Math #:", " " + math);
            telemetry.addData("Loop #:", " " + loop);
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }

