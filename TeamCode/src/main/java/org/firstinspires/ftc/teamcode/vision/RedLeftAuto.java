/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.PIDController;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous
public class RedLeftAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PIDController pid;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftTArm = null;
    private DcMotor rightTArm = null;
    private DcMotor leftBArm = null;
    private DcMotor rightBArm = null;
    private CRServo claw= null;
    private Servo clawSpinner = null;
    private BNO055IMU imu = null;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C270 at default res
    // You will need to do your own calibration for other configurations!
    double fx = 951.1737219669708;
    double fy = 946.8567516102694;
    double cx = 366.70168128710145;
    double cy = 235.8132829164716;

    double p = 0.01;
    double i = 0.0;
    double d = 0.0;

    double robotHeading = 0;
    Orientation lastAngles = new Orientation();

    // UNITS ARE METERS
    double tagsize = 0.04;

    int phase = 0;
    int c = 0;
    int counter = 0;

    int ID_TAG_OF_INTEREST = 3; // Tag ID 18 from the 36h11 family
    ArrayList<Integer> TARGET_TAGS = new ArrayList<Integer>(3);

    AprilTagDetection tagOfInterest = null;

    public enum Direction {
        FORWARDS, BACKWARDS, LEFT, RIGHT, ROTATE_LEFT, ROTATE_RIGHT, ROTATE_PID
    }
    @Override
    public void runOpMode()
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front left drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back left drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right drive");
        leftTArm = hardwareMap.get(DcMotor.class, "left top arm");
        rightTArm = hardwareMap.get(DcMotor.class, "right top arm");
        leftBArm = hardwareMap.get(DcMotor.class, "left bottom arm");
        rightBArm = hardwareMap.get(DcMotor.class, "right bottom arm");
        claw = hardwareMap.get(CRServo.class, "claw");
        clawSpinner = hardwareMap.get(Servo.class, "claw spinner");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightTArm.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBArm.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftTArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //TODO: make a
        TARGET_TAGS.add(1);
        TARGET_TAGS.add(2);
        TARGET_TAGS.add(3);

        pid = new PIDController(p, i, d);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        ArrayList<AprilTagDetection> currentDetections;
        while (opModeIsActive() && c == 0) {
            currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 && (pid.getSetpoint() == 0 || phase == 0)) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (TARGET_TAGS.contains(tag.id)) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag Found!");
                    tagToTelemetry(tagOfInterest);
                    c++;
                } else {
                    telemetry.addLine("No Tags Found!");
                }
                telemetry.update();
                sleep(20);
            }
        }

        telemetry.clearAll();

        while (opModeIsActive() && c != 0) {
            // Does the code
            if(tagOfInterest == null)
            {
                telemetry.addLine("No tags!");
            } else {
                switch (phase) {
                    case 0:
                        pid.setSetpoint(-62.5);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 1:
                        driveToPosition(4026, 0.5, Direction.FORWARDS);
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 2:
                        //TODO: code the grabbing of the cone
                        claw.setPower(-0.9);
                        moveArm(0.75, 1080);
                        driveToPosition(360, 0.5, Direction.FORWARDS);
                        if (!leftBackDrive.isBusy()) {
                            claw.setPower(0.8);
                            moveArm(0.35, 2160); //TODO: get this value
                            phase++;
                        }
                        break;
                    case 3:
                        //square up
                        pid.setSetpoint(90);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate((getAngle())), false);
                        } else {
                            mecanumDrive(Direction.BACKWARDS, 1.0, false);
                            sleep(500);
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 4:
                        //rotate to pole
                        pid.setSetpoint(90); //TODO: get angle
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        } else {
                            stopAndResetEncoders();
                            if (counter == 4) {
                                phase = 12;
                            } else if (counter == 5) {
                                phase = 16;
                            } else {
                                phase++;
                            }
                        }
                        break;
                    case 5:
                        //go to pole
                        driveToPosition(3555, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 6:
                        claw.setPower(-0.9);
                        driveToPosition(-360, 0.5, Direction.BACKWARDS);
                        if (!leftFrontDrive.isBusy()) {
                            moveArm(-0.35, 3240);
                            phase++;
                        }
                        break;
                    case 7:
                        pid.setSetpoint(-90);
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 8:
                        driveToPosition(10, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 9:
                        //turn to cone pile again
                        pid.setSetpoint(20); //TODO: get angle
                        if (!pid.atSetpoint()) {
                            mecanumDrive(Direction.ROTATE_PID, pid.calculate(getAngle()), false);
                        } else {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 10:
                        if (TARGET_TAGS.get(2) == tagOfInterest.id) {
                            //tag 3
                            phase = 4;
                            counter = 4;
                        } else if (TARGET_TAGS.get(1) == tagOfInterest.id && counter == 2) {
                            //tag 2
                            phase = 4;
                            counter = 5;
                        } else if (TARGET_TAGS.get(0) == tagOfInterest.id && counter == 3) {
                            //tag 1
                            phase = 19;
                        } else {
                            phase = 2;
                            counter++;
                        }
                        break;
                    case 12:
                        //go to center of patch
                        driveToPosition(3600, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase = 69;
                        }
                        break;
                    case 16:
                        //go to center of patch
                        driveToPosition(5389, 0.75, Direction.FORWARDS); //TODO: get values
                        if (!leftFrontDrive.isBusy()) {
                            stopAndResetEncoders();
                            phase++;
                        }
                        break;
                    case 19:
                        //we're in location 1 already lmao
                        phase = 69;
                        break;
                    default:
                        telemetry.addLine("All done!");
                        break;
                }
            }
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void PIDToTelemetry(PIDController pid) {
        telemetry.addLine("PID Values")
                .addData("P", pid.getP())
                .addData("At Setpoint?", pid.atSetpoint())
                .addData("setpoint", pid.getSetpoint());
    }


    void mecanumDrive(Direction direction, double power, boolean pos) {
        // leftFrontDrive.setPower();
        // rightFrontDrive.setPower();
        // leftBackDrive.setPower();
        // rightBackDrive.setPower();
        if (!pos) {
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        switch (direction) {
            case FORWARDS:
                leftFrontDrive.setPower(Math.abs(power));
                rightFrontDrive.setPower(Math.abs(power));
                leftBackDrive.setPower(Math.abs(power));
                rightBackDrive.setPower(Math.abs(power));
                break;
            case BACKWARDS:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case LEFT:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, 1));
                leftBackDrive.setPower(Math.copySign(power, 1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case RIGHT:
                leftFrontDrive.setPower(Math.copySign(power, 1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, 1));
                break;
            case ROTATE_LEFT:
                leftFrontDrive.setPower(Math.copySign(power, -1));
                rightFrontDrive.setPower(Math.copySign(power, 1));
                leftBackDrive.setPower(Math.copySign(power, -1));
                rightBackDrive.setPower(Math.copySign(power, 1));
                break;
            case ROTATE_RIGHT:
                leftFrontDrive.setPower(Math.copySign(power, 1));
                rightFrontDrive.setPower(Math.copySign(power, -1));
                leftBackDrive.setPower(Math.copySign(power, 1));
                rightBackDrive.setPower(Math.copySign(power, -1));
                break;
            case ROTATE_PID:
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
                break;
            default:
                telemetry.addLine("L + Ratio + You fell off + don't care + didn't ask + you're british + anime pfp");
                break;
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
    void stopAndResetEncoders() {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(0);
        rightFrontDrive.setTargetPosition(0);
        rightBackDrive.setTargetPosition(0);
        leftBackDrive.setTargetPosition(0);
    }
    void driveToPosition(int target, double power, Direction direction) {
        if (leftBackDrive.getTargetPosition() != target) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFrontDrive.setTargetPosition(target);
            rightFrontDrive.setTargetPosition(target);
            rightBackDrive.setTargetPosition(target);
            leftBackDrive.setTargetPosition(target);
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
        }
        mecanumDrive(direction, power, true);
    }
    void stopMotors() {
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void moveArm(double power, int position) {
        leftBArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftTArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBArm.setTargetPosition(position);
        leftTArm.setTargetPosition(position);
        rightTArm.setTargetPosition(position);
        rightBArm.setTargetPosition(position);
        leftBArm.setPower(power);
        leftTArm.setPower(power);
        rightTArm.setPower(power);
        rightBArm.setPower(power);
    }
}