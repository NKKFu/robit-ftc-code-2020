package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "Skystone", group = "TeleOp")
public class HopeJ extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MapHardware();

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //encoderDrive(1,1, 0.5);
        TeleOp();
    }

    private Servo servo_0, servo_1, servo_2;
    private DcMotor rightDrive, leftDrive;
    private DcMotor liftMotor;

    private void MapHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        servo_0 = hardwareMap.get(Servo.class, "servo_0");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");
    }

    // TELEOPERADO ========================================================
    private void TeleOp() {
        waitForStart();

        boolean lastA = false;
        while(opModeIsActive()){
            // ROTINA DE 180 GRAUS
            if (gamepad1.left_trigger > 0 || gamepad2.left_trigger > 0) {
                leftDrive.setPower(1);
                rightDrive.setPower(-1);
                sleep(900);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            } else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                leftDrive.setPower(-1);
                rightDrive.setPower(1);
                sleep(900);
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            if(gamepad1.a || gamepad2.right_trigger > 0) {
                while(gamepad1.a || gamepad2.right_trigger > 0) {
                    if (isStopRequested())
                        break;
                }
                lastA = !lastA;
                ServosPosition (lastA);
            }

            if (gamepad2.right_bumper) {
                SetLiftPosition(0);
            } else if (gamepad1.x || gamepad2.x) {
                SetLiftPosition(1);
            } else if (gamepad1.y || gamepad2.y) {
                SetLiftPosition(2);
            } else if (gamepad1.b || gamepad2.b) {
                SetLiftPosition(3);
            }

            if ((gamepad1.dpad_up) && liftMotor.getCurrentPosition() < 3960) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(1);
            } else if (gamepad1.dpad_down && (liftMotor.getCurrentPosition() > 0)) {
                liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftMotor.setPower(-1);
            } else {
                if (liftMotor.getCurrentPosition() < 0) {
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(0.3);
                } else {
                    liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMotor.setPower(0);
                }
            }

            if (gamepad1.dpad_left) {
                servo_2.setPosition(-1);
            } else if (gamepad1.dpad_right){
                servo_2.setPosition(1);
            } else {
                servo_2.setPosition(0.5);
            }

            ControlMovement();

            telemetry.addData("Lift Motor: ", liftMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void SetLiftPosition (int pos) {
        switch (pos) {
            default:
                liftMotor.setTargetPosition(0);
                break;
            case 1:
                liftMotor.setTargetPosition(840);
                break;
            case 2:
                liftMotor.setTargetPosition(2100);
                break;
            case 3:
                liftMotor.setTargetPosition(3000);
                break;
        }

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(1);
        while (liftMotor.isBusy()) { ControlMovement(); }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void ControlMovement() {
        boolean isTurboActive   = gamepad1.right_bumper;
        double speed            = (isTurboActive ? 1 : 0.5);

        double Linput_player1   = (-gamepad1.left_stick_y + gamepad1.right_stick_x);
        double Rinput_player1   = (-gamepad1.left_stick_y - gamepad1.right_stick_x);
        double Linput_player2   = (-gamepad2.left_stick_y + gamepad2.right_stick_x);
        double Rinput_player2   = (-gamepad2.left_stick_y - gamepad2.right_stick_x);

        leftDrive.setPower(speed * (Linput_player1 + Linput_player2));
        rightDrive.setPower(speed * (Rinput_player1 + Rinput_player2));
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
}