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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="PID Tuning", group="Linear Opmode")
//@Disabled
public class PIDTuning extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftTArm = null;
    private DcMotor rightTArm = null;
    private DcMotor leftBArm = null;
    private DcMotor rightBArm = null;
    private PIDController pid = null;
    double robotHeading = 0;
    Orientation lastAngles = new Orientation();
    private BNO055IMU imu = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front left drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back left drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right drive");
        leftTArm = hardwareMap.get(DcMotor.class, "left top arm");
        rightTArm = hardwareMap.get(DcMotor.class, "right top arm");
        leftBArm = hardwareMap.get(DcMotor.class, "left bottom arm");
        rightBArm = hardwareMap.get(DcMotor.class, "right bottom arm");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightTArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftTArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        pid = new PIDController(0.25, 0.0, 0.0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            pid.setTolerance(0.1);

            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            if (gamepad1.left_trigger > 0.0) {
                leftTArm.setPower(gamepad1.left_trigger);
                rightTArm.setPower(gamepad1.left_trigger);
                leftBArm.setPower(gamepad1.left_trigger);
                rightBArm.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0.0) {
                leftTArm.setPower(gamepad1.right_trigger * -1);
                rightTArm.setPower(gamepad1.right_trigger * -1);
                leftBArm.setPower(gamepad1.right_trigger * -1);
                rightBArm.setPower(gamepad1.right_trigger * -1);
            } else {
                leftTArm.setPower(0);
                rightTArm.setPower(0);
                leftBArm.setPower(0);
                rightBArm.setPower(0);
            }

            if (gamepad1.dpad_up) {
                pid.setP((pid.getP() + 0.01));
                sleep(500);
            } else if (gamepad1.dpad_down) {
                pid.setP((pid.getP() - 0.01));
                sleep(500);
            } else if (gamepad1.dpad_left) {
                pid.setP((pid.getP() + 0.001));
                sleep(500);
            } else if (gamepad1.dpad_right) {
                pid.setP((pid.getP() - 0.001));
                sleep(500);
            }

            if (gamepad1.a) {
                pid.setSetpoint(0);
                double power = pid.calculate(getAngle());
                if (power > 0) {
                    leftFrontDrive.setPower(Math.copySign(power, -1));
                    rightFrontDrive.setPower(Math.copySign(power, 1));
                    leftBackDrive.setPower(Math.copySign(power, -1));
                    rightBackDrive.setPower(Math.copySign(power, 1));
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1));
                    rightFrontDrive.setPower(Math.copySign(power, -1));
                    leftBackDrive.setPower(Math.copySign(power, 1));
                    rightBackDrive.setPower(Math.copySign(power, -1));
                }
            } else if (gamepad1.b) {
                pid.setSetpoint(90);
                double power = pid.calculate(getAngle());
                if (power > 0) {
                    leftFrontDrive.setPower(Math.copySign(power, -1));
                    rightFrontDrive.setPower(Math.copySign(power, 1));
                    leftBackDrive.setPower(Math.copySign(power, -1));
                    rightBackDrive.setPower(Math.copySign(power, 1));
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1));
                    rightFrontDrive.setPower(Math.copySign(power, -1));
                    leftBackDrive.setPower(Math.copySign(power, 1));
                    rightBackDrive.setPower(Math.copySign(power, -1));
                }
            } else if (gamepad1.x) {
                pid.setSetpoint(-90);
                double power = pid.calculate(getAngle());
                if (power > 0) {
                    leftFrontDrive.setPower(Math.copySign(power, -1));
                    rightFrontDrive.setPower(Math.copySign(power, 1));
                    leftBackDrive.setPower(Math.copySign(power, -1));
                    rightBackDrive.setPower(Math.copySign(power, 1));
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1));
                    rightFrontDrive.setPower(Math.copySign(power, -1));
                    leftBackDrive.setPower(Math.copySign(power, 1));
                    rightBackDrive.setPower(Math.copySign(power, -1));
                }
            } else if (gamepad1.y) {
                pid.setSetpoint(180);
                double power = pid.calculate(getAngle());
                if (power > 0) {
                    leftFrontDrive.setPower(Math.copySign(power, -1));
                    rightFrontDrive.setPower(Math.copySign(power, 1));
                    leftBackDrive.setPower(Math.copySign(power, -1));
                    rightBackDrive.setPower(Math.copySign(power, 1));
                } else if (power < 0) {
                    leftFrontDrive.setPower(Math.copySign(power, 1));
                    rightFrontDrive.setPower(Math.copySign(power, -1));
                    leftBackDrive.setPower(Math.copySign(power, 1));
                    rightBackDrive.setPower(Math.copySign(power, -1));
                }
            }

            if (gamepad1.left_bumper) {
                leftFrontDrive.setTargetPosition(1440);
                rightFrontDrive.setTargetPosition(1440);
                rightBackDrive.setTargetPosition(1440);
                leftBackDrive.setTargetPosition(1440);
                if (    !(leftFrontDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                        !(rightFrontDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                        !(leftBackDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION) ||
                        !(rightBackDrive.getMode() == DcMotor.RunMode.RUN_TO_POSITION)
                ) {
                    leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                leftFrontDrive.setPower(0.5);
                rightFrontDrive.setPower(0.5);
                leftBackDrive.setPower(0.5);
                rightBackDrive.setPower(0.5);

            } else if (gamepad1.right_bumper) {
                leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("Motors")
                    .addData("Front Left (%.2f)", frontLeftPower)
                    .addData("Front Right (%.2f)", frontRightPower)
                    .addData("Back Left (%.2f)", backLeftPower)
                    .addData("Back Right (%.2f)", backRightPower)
                    .addData("arm power (%.2f)", leftTArm.getPower())
                    .addData("p", pid.getP())
                    .addData("gyro", getAngle())
                    .addData("motor pos", leftBackDrive.getCurrentPosition());
            telemetry.addLine("note to self: save p value b4 killing program");
            telemetry.update();
        }
    }
    void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotHeading = 0;
    }
    // because I can't find any docs on the gyro and how it works, im ctrl + c and ctrl + v ing code
    // and so the ppl i copied from have angles set to + and - 180
    // not 360
    // because imu idfk
    double getAngle() {
        Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = currentAngle.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        robotHeading += deltaAngle;

        lastAngles = currentAngle;
        return robotHeading;
    }
}
