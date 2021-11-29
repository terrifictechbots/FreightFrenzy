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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

//import org.firstinspires.ftc.Terrycontroller.external.samples.HardwarePushbot;

@Autonomous(name="Blue Camera Duck Drop", group="Linear Opmode")
//@Disabled
public class Blue_Camera_Duck_Drop extends LinearOpMode {

    /* Declare OpMode members. */
    KellyHardware Kelly = new KellyHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder - 1440
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.5;
    static final double SLIDEL_SPEED = 0.8;
    static final double SLIDER_SPEED = 0.8;
    static final double ARM_SPEED = 1;
    static final double STOP = 0;

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    // Temporary variable to hold the label of the detected object
    double tempAngle = 0;

    private static final String VUFORIA_KEY =
            "ATFRicb/////AAABmZn2su26aEEOs4UYu6C/rNhH0kr3n7JPTRd0WBXAPrZ6dA+lP2RUQxhaRc0thpW5l8VJiq0bpggp+4RKM++qtmzR+gK454s+3LXaYDPJ4gvkZfNpzd25wqGOKMJDbMnBb3vMNNgMLGyENiaK/J7tMTVFkPSx5mQxGPhoaARxt8pAx9i3XjOMgL+h1ClMIIg7o9DgaSxFn+9eFppEADjmhfZi3BZoYSUlkYDhJnseH/64lGWUWVuscRAfWeeG2+cDMDwSRp3Qhgg9wcZYzRbnQ7Z1vcEUIwtZeKox1ZU0YKhEWSJiRFBwuYCNZbn/hbXSji9bx0NIDhZb1o9zPQhpKp8ehlWD8/mx0Txzx+f1BOKq";
    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.9, 16.0 / 9.0);
        }

            if (opModeIsActive()) {
                while (opModeIsActive()&&(runtime.seconds()<6)) {
                    if (tfod != null) {
                        // getUpdatedRecognitions() will return null if no new information is available since
                        // the last time that call was made.
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());
                                telemetry.addData(String.format("  angle (%d)", i), "%.03f ",
                                        recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                i++;
                                tempAngle= recognition.estimateAngleToObject(AngleUnit.DEGREES);
                               // telemetry.addData(String.format("templabel (%d)", i), tempLabel);
                            }
                            telemetry.update();
                        }
                    }
                }
            }

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
        //slide away from wall
        encoderDrive(SLIDER_SPEED / 2, 7, -7, -7, 7, 0);

        //Drive back until duck wheel is touching side of duck carousel
        encoderDrive(DRIVE_SPEED / 2, -33.5, -33.5, -33.5, -33.5, 0);

        //To deliver duck, spin duck carousel until duck falls off onto mat
        Kelly.runtime.reset();
        while (opModeIsActive() && (Kelly.runtime.seconds() < 2.5)) {
            Kelly.duckArmDrive.setPower(1);
        }
        Kelly.duckArmDrive.setPower(0);

        //Slide right in line with shipping hub
        encoderDrive(SLIDER_SPEED, 32, -32, -32, 32, 0);
        //lift pickup arm up to put block on top level
        //first position = -13, second position = 7 (or 10)
        if (tempAngle >0) {
            encoderDrive(ARM_SPEED, 0, 0, 0, 0, -.3);
            encoderDrive(DRIVE_SPEED, 36, 36, 36, 36, 0);
            } else if (tempAngle <0) {
            encoderDrive(ARM_SPEED, 0, 0, 0, 0, -1);
            encoderDrive(DRIVE_SPEED, 36, 36, 36, 36, 0);
        } else {
            encoderDrive(ARM_SPEED, 0, 0, 0, 0, -.1);
            encoderDrive(DRIVE_SPEED, 32, 32, 32, 32, 0);
        }
            encoderDrive(1,1,1,1,1,0);
        encoderDrive(ARM_SPEED / 4, 0, 0, 0, 0, .7);
       // encoderDrive(ARM_SPEED, 0, 0, 0, 0, -1);
      //  encoderDrive(ARM_SPEED, 0, 0, 0, 0, -1);
        //go forward until next to shipping hub
        /*encoderDrive(DRIVE_SPEED, 36, 36, 36, 36, 0);
        //let go of servo, dropping block onto highest level
        Kelly.servoPickupClamp.setPosition(0);
        sleep(1000);
        //move back from shipping hub
        encoderDrive(DRIVE_SPEED, -36, -36, -36, -36, 0);
        //move arm back down
        encoderDrive(ARM_SPEED / 4, 0, 0, 0, 0, .7);
        sleep(500);
        //slide into storage unit
        encoderDrive(SLIDER_SPEED, -7, 7, 7, -7, 0);*/

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
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}

