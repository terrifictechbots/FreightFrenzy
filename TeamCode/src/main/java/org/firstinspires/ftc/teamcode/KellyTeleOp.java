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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//We're testing this comment

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

@TeleOp(name="KellyTeleOp", group="Linear Opmode")
//@Disabled
public class KellyTeleOp extends LinearOpMode {

    // Declare OpMode members.
    private KellyHardware Kelly = new KellyHardware();
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        //System.out.println("Is this thing working?!");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        Kelly.init(hardwareMap);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        // Send telemetry message to signify Kelly waiting;


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double slidePower;
            double drivePower;
            double spinPowerCCW;
            double slideSpower;
            double spinPowerCW;
            double driveSPower;
            double duckspinPower;


            slidePower = gamepad1.right_stick_x;
            drivePower = gamepad1.right_stick_y;
            driveSPower = gamepad1.left_stick_y;
            spinPowerCCW = -gamepad1.left_trigger;
            slideSpower = gamepad1.left_stick_x;
            spinPowerCW = gamepad1.right_trigger;
            duckspinPower = gamepad2.right_trigger;

            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.

            telemetry.update();

            if (gamepad2.right_bumper) {
                Kelly.servoPickupClamp.setPosition(0);
            } else {
                Kelly.servoPickupClamp.setPosition(1);
            }


            // Send calculated power to wheels

            if (gamepad1.right_stick_x > 0.2 || (gamepad1.right_stick_x < -0.2)) {
                Kelly.slideL(slidePower / 2);
            } else if (gamepad1.right_stick_y > 0.2 || (gamepad1.right_stick_y < -0.2)) {
                Kelly.drive(drivePower/2);
            } else if (gamepad1.left_trigger > 0.2 || (gamepad1.left_trigger < -0.2)) {
                Kelly.spin(spinPowerCCW / 2);
            } else if (gamepad1.left_stick_x > 0.2 || (gamepad1.left_stick_x < -0.2)) {
                Kelly.slideS(slideSpower);
            } else if (gamepad1.right_trigger > 0.2 || (gamepad1.right_trigger < -0.2)) {
                Kelly.spin(spinPowerCW / 2);
            } else if (gamepad1.left_stick_y > 0.2 || (gamepad1.left_stick_y < -0.2)) {
                Kelly.driveS(driveSPower*2);
            } else {
                Kelly.drive(0);
                Kelly.slideL(0);
                Kelly.spin(0);
                Kelly.spin(0);
                Kelly.driveS(0);
                Kelly.slideS(0);
            }

            Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (gamepad2.left_stick_y > 0.2) {
                Kelly.pickupArmDrive.setPower(-.2);
            } else if (gamepad2.left_stick_y < 0.2 && gamepad2.left_stick_y > -0.2) {
                Kelly.pickupArmDrive.setPower(0);
            }

            if (gamepad2.left_stick_y < -0.2) {
                Kelly.pickupArmDrive.setPower(-1);
            } else if (gamepad2.left_stick_y < 0.2 && gamepad2.left_stick_y > -0.2) {
                Kelly.pickupArmDrive.setPower(0);
            }


            if(gamepad2.right_trigger > 0.2) {
                Kelly.duckArmDrive.setPower(1);
                Kelly.rightduckArmDrive.setPower(1);
            } else if (gamepad2.right_trigger < 0.2 && gamepad2.right_trigger > -0.2) {
                Kelly.rightduckArmDrive.setPower(0);
                Kelly.duckArmDrive.setPower(0);
            }
            }
        }
    }