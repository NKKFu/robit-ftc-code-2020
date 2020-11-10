package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "DEBUG_MODE", group = "Debug")
public class DEBUG extends LinearOpMode {

    private Servo servo_0, servo_1;
    private DcMotor rightDrive, leftDrive;
    private DcMotor liftMotor;

    private BNO055IMU imu;
    private int AngleTarget = 0;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        MapHardware();
        initIMU();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Left Drive ", leftDrive.getCurrentPosition());
            telemetry.addData("Right Drive ", rightDrive.getCurrentPosition());
            telemetry.addData("Lift Motor ", liftMotor.getCurrentPosition());

            telemetry.addData("Servo 0 ", servo_0.getPosition());
            telemetry.addData("Servo 1 ", servo_1.getPosition());

            telemetry.addData("IMU Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES));

            telemetry.update();
        }
    }

    private void MapHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        servo_0 = hardwareMap.get(Servo.class, "servo_0");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    private void encoderDrive(double left, double right, double velocity) {
        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (int) (left * 220));
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (int) (right * 220));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(velocity);
        rightDrive.setPower(velocity);

        while (leftDrive.isBusy() && rightDrive.isBusy()) {
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void ServosPosition(boolean open) {
        if (open) { // Abrir
            servo_0.setPosition(0.5);
            servo_1.setPosition(-1);
        } else {
            servo_0.setPosition(-0.05);
            servo_1.setPosition(0.4);
        }
    }

    // Orientation angles =
    //      imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    private void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 10);
    }
}