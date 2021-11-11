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
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S ENT RIGHTS ARE GRANTED BY THIS
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.Terrycontroller.external.samples.HardwarePushbot;

@Autonomous(name="Red Drop Warehouse Park", group="Linear Opmode")
//@Disabled
public class Red_Drop_Warehouse_Park extends LinearOpMode {

    /* Declare OpMode members. */
    KellyHardware Kelly   = new KellyHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder - 1440
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.8;
    static final double     TURN_SPEED              = 0.5;
    static final double SLIDEL_SPEED = 0.8;
    static final double SLIDER_SPEED = 0.8;
    static final double ARM_SPEED = 1;
    static final double STOP = 0;


    @Override
    public void runOpMode() {

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Kelly.init(hardwareMap);
        // Start using camera to look for rings

        // Send telemetry message to signify Terry waiting;
        telemetry.addData("Status", "Resetting encoder");
        telemetry.update();

        Kelly.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Turn On RUN_TO_POSITION
        Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //start with back facing warehouse
       // Kelly.servoPickupClamp.setPosition(1);
        //lift pickup arm up to put block on top level
        encoderDrive(ARM_SPEED,0,0,0,0,-1.2);
        // Slide until in line with shipping hub
        encoderDrive(SLIDER_SPEED,43,-43,-43,43,0);
        //go forward until next to shipping hub
        encoderDrive(DRIVE_SPEED,7,7,7,7,0);
        //let go of servo, dropping block onto highest level
        Kelly.servoPickupClamp.setPosition(0);
        sleep(1000);
        //move back from shipping hub
        encoderDrive(DRIVE_SPEED,-9,-9,-9,-9,0);
        //move arm back down
        encoderDrive(ARM_SPEED/4,0,0,0,0,.7);
        sleep(500);
        //slide away from shipping hub
        encoderDrive(SLIDEL_SPEED,-15,15,15,-15,0);
        //drive forward to gain momentum for driving back over barriers into warehouse
        encoderDrive(DRIVE_SPEED,10,10,10,10,0);
        //drive back, over barriers, into warehouse to park
        encoderDrive(DRIVE_SPEED*1.25,-50,-50,-50,-50,0);

            Kelly.leftDrive.setPower(0);
            Kelly.rightDrive.setPower(0);
            Kelly.leftBackDrive.setPower(0);
            Kelly.rightBackDrive.setPower(0);
            Kelly.pickupArmDrive.setPower(0);
        }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches, double armInches) {
                int newLeftTarget;
                int newRightTarget;
                int newLeftBackTarget;
                int newRightBackTarget;
                int newArmTarget;

                // Ensure that the opmode is still active
                if (opModeIsActive()) {

                    // Determine new target position, and pass to motor controller
                    newLeftTarget = Kelly.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                    newRightTarget = Kelly.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                    newLeftBackTarget = Kelly.leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
                    newRightBackTarget = Kelly.rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);
                    newArmTarget = Kelly.pickupArmDrive.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH);


                    Kelly.leftDrive.setTargetPosition(newLeftTarget);
                    Kelly.rightDrive.setTargetPosition(newRightTarget);
                    Kelly.leftBackDrive.setTargetPosition(newLeftBackTarget);
                    Kelly.rightBackDrive.setTargetPosition(newRightBackTarget);
                    Kelly.pickupArmDrive.setTargetPosition(newArmTarget);

                    Kelly.pickupArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // reset the timeout time and start motion.
                    runtime.reset();
                    Kelly.leftDrive.setPower(Math.abs(speed));
                    Kelly.rightDrive.setPower(Math.abs(speed));
                    Kelly.leftBackDrive.setPower(Math.abs(speed));
                    Kelly.rightBackDrive.setPower(Math.abs(speed));
                    Kelly.pickupArmDrive.setPower(Math.abs(speed));

                    while (opModeIsActive() && (Kelly.rightDrive.isBusy() || Kelly.leftBackDrive.isBusy() || Kelly.leftDrive.isBusy() || Kelly.leftBackDrive.isBusy()||Kelly.pickupArmDrive.isBusy()))
                    {

                        // Display it for the driver.
                        telemetry.addData("TargetPos", "Running to %7d :%7d :%7d :%7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
                        telemetry.addData("CurrentPos", "Running at %7d :%7d :%7d :%7d",
                                Kelly.leftDrive.getCurrentPosition(),
                                Kelly.rightDrive.getCurrentPosition(),
                                Kelly.leftBackDrive.getCurrentPosition(),
                                Kelly.rightBackDrive.getCurrentPosition());
                        telemetry.update();
                    }

                    Kelly.leftDrive.setPower(0);
                    Kelly.rightDrive.setPower(0);
                    Kelly.leftBackDrive.setPower(0);
                    Kelly.rightBackDrive.setPower(0);
                    Kelly.pickupArmDrive.setPower(-.2);

                    Kelly.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Kelly.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Kelly.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    Kelly.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Kelly.pickupArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    // Turn On RUN_TO_POSITION
                    Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                }
       /* public void encoderArmDrive(double armspeed,double armInches) {
            int newArmTarget;

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newArmTarget = Kelly.pickupArmDrive.getCurrentPosition() + (int)(armInches * COUNTS_PER_INCH);
                Kelly.pickupArmDrive.setTargetPosition(newArmTarget);

                // Turn On RUN_TO_POSITION
                Kelly.pickupArmDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();
                Kelly.pickupArmDrive.setPower(Math.abs(speed));
                while (opModeIsActive() &&
                        (Kelly.pickupArmDrive.isBusy())) {
                }

                // Stop all motion;
                Kelly.pickupArmDrive.setPower(-.2);

                // Turn off RUN_TO_POSITION
                Kelly.pickupArmDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }*/
    }
}

