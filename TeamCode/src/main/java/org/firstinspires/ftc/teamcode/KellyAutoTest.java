/* Terrific Techbots FTC #14563 2021-2022 Autonomous Code */


package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
 *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Kelly AutoTest", group="Terry")
//@Disabled
public class KellyAutoTest extends LinearOpMode {

    // The IMU sensor object
   // BNO055IMU imu;

    // State used for updating telemetry
//    Orientation angles;
//    Acceleration gravity;

    /* Declare OpMode members. */
    KellyHardware Kelly   = new KellyHardware();   // Use Kelly's hardware


    /*static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder - 1440
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
*/

    @Override
    public void runOpMode() {
        //*code from imu sample code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */

        Kelly.init(hardwareMap);

        Kelly.imu = hardwareMap.get(BNO055IMU.class, "imu");
        Kelly.imu.initialize(parameters);
//end code from imu sample code\

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.

        Kelly.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

//        Terry.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Terry.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Terry.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        Terry.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        // Step through each leg of the path,
        // Put a hold after each turn
        Kelly.gyroDrive(Kelly.DRIVE_SPEED,57,0.0);
        Kelly.gyroDrive(Kelly.DRIVE_SPEED,-57,0);
        Kelly.gyroDrive(Kelly.DRIVE_SPEED, 5.0, 0.0);    // Drive FWD 5 inches
        Kelly.gyroSlideR(Kelly.SLIDE_SPEED, 24.0, 0.0);  //slide right 24 inches
        Kelly.gyroHold(Kelly.TURN_SPEED,0,2);
        Kelly.gyroSlideL(Kelly.SLIDE_SPEED, 45.0, 0.0);  //slide left 45 inches
        Kelly.gyroHold(Kelly.TURN_SPEED,0,2);
        Kelly.gyroDrive(Kelly.DRIVE_SPEED,10,0); //drive forward 10 inches
        Kelly.gyroHold(Kelly.TURN_SPEED,0,2);
        Kelly.gyroSlideR(Kelly.SLIDE_SPEED, 48.0, 0.0);  //slide right 48 inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }}

   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading

   public void gyroDrive ( double speed, double distance, double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double leftBackSpeed;
        double rightBackSpeed;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = Kelly.leftDrive.getCurrentPosition() + moveCounts;
        newRightTarget = Kelly.rightDrive.getCurrentPosition() + moveCounts;
        newLeftBackTarget = Kelly.leftBackDrive.getCurrentPosition() + moveCounts;
        newRightBackTarget = Kelly.rightBackDrive.getCurrentPosition() + moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        Kelly.leftDrive.setTargetPosition(newLeftTarget);
        Kelly.rightDrive.setTargetPosition(newRightTarget);
        Kelly.leftBackDrive.setTargetPosition(newLeftBackTarget);
        Kelly.rightBackDrive.setTargetPosition(newRightBackTarget);

        Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        Kelly.leftDrive.setPower(speed);
        Kelly.rightDrive.setPower(speed);
        Kelly.leftBackDrive.setPower(speed);
        Kelly.rightBackDrive.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while (Kelly.leftDrive.isBusy() && Kelly.rightDrive.isBusy() && Kelly.leftBackDrive.isBusy() && Kelly.rightBackDrive.isBusy()) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            Kelly.leftDrive.setPower(leftSpeed);
            Kelly.rightDrive.setPower(rightSpeed);
            Kelly.leftBackDrive.setPower(leftSpeed);
            Kelly.rightBackDrive.setPower(rightSpeed);

            // Display drive status for the driver.
            telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
            telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget, newLeftBackTarget, newRightBackTarget);
            telemetry.addData("Actual",  "%7d:%7d",      Kelly.leftDrive.getCurrentPosition(),
                    Kelly.rightDrive.getCurrentPosition(), Kelly.leftBackDrive.getCurrentPosition(), Kelly.rightBackDrive.getCurrentPosition());
            telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
            telemetry.update();
        }

        // Stop all motion;
        Kelly.leftDrive.setPower(0);
        Kelly.rightDrive.setPower(0);
        Kelly.leftBackDrive.setPower(0);
        Kelly.rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        Kelly.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Kelly.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (!onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (holdTimer.time() < holdTime) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        Kelly.leftDrive.setPower(0);
        Kelly.rightDrive.setPower(0);
        Kelly.leftBackDrive.setPower(0);
        Kelly.rightBackDrive.setPower(0);
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double leftBackSpeed;
        double rightBackSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            leftBackSpeed  = 0.0;
            rightBackSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
            rightBackSpeed  = speed * steer;
            leftBackSpeed   = -rightBackSpeed;
        }

        // Send desired speeds to motors.
        Kelly.leftDrive.setPower(leftSpeed);
        Kelly.rightDrive.setPower(rightSpeed);
        Kelly.leftBackDrive.setPower(leftBackSpeed);
        Kelly.rightBackDrive.setPower(rightBackSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed, leftBackSpeed, rightBackSpeed);

        return onTarget;
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public void gyroSlideR (double speed, double distance, double angle) {
        Kelly.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly.leftDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.gyroDrive(speed, distance, angle);
        Kelly.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.leftDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void gyroSlideL (double speed, double distance, double angle) {
        Kelly. rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly.leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly.rightDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly. leftDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.gyroDrive(speed, distance, angle);
        Kelly.rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        Kelly.rightDrive.setDirection(DcMotor.Direction.FORWARD);
        Kelly. leftDrive.setDirection(DcMotor.Direction.FORWARD);
    }

//end code from imu sample code*/
