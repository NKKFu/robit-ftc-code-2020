package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

@Autonomous(name = "Em Sala 1", group = "Auto")
public class SkystoneCode extends LinearOpMode {

    // * * Configurações * * //
    private double ROTATION_AMOUNT = 0.92;
    private double ROTATION_VELOCITY = 1;

    private double NAVIGATION_SPEED = 0.8;
    private double FORWARD_MAX_SPEED = 1;

    private int TIME_WAIT = 0;

    // * * Componentes do Hardware * * //
    private Servo servo_0, servo_1, servo_2;
    private DcMotor rightDrive, leftDrive;
    private DcMotor liftMotor;

    private BNO055IMU imu;
    private int AngleTarget = 0;

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        MapHardware();
        initIMU();

        // Configuração dos motores //
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Parâmetros para a inicialização do Vuforia //
        VuforiaLocalizer.Parameters parameters =
                new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "ATX1WMH/////AAABmZRCfAn9XENvtus7/8vEDzAjZUqBTtwyQRDZXSSxrKinG+QxvA51PBQf+MmhxVq5dnpIJ5pQcu8NIAo2ZJdWxJis8ws4Mx9efHxDgVdU4bwtKuOPmlUvdtwLHw8QZXpUA3nWj5G6HW2mX/7aE2hpj3sMsjjQKJMd7ify6hnYkVZwx3Ej1mODMH6FQ9UQTQuTJYN4vUGZQ3ggd/Yh+j1n+eldooxfN3G9SGh2eaa2vvfIsAsrZ7yrgHZUzw/fPDFa3MIk6lML0L02lKeksYGXqStHnzVJOiL/v2XNj+LmwpIvwWiijR4eTK3wl6oXjD0NP3i3bO37veiYrA0lTB+MZ18KpHQYiONQQ8GtTaP1eg3r";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);

        // Enquanto o robô está andando, o Vuforia está sendo inciializado assim evitando perda de
        // tempo desnecessário

        // Liga o flash da câmera
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        VuforiaTrackables skystones = vuforia.loadTrackablesFromAsset("Skystone");
        skystones.get(0).setName("SKYSTONE");
        skystones.activate();

        CameraDevice.getInstance().setFlashTorchMode(true);

        // * * FINALIZOU A INICIALIZAÇÃO DO VUFORIA * * //

        // PROCURA A SKYSTONE #1
        boolean getout = false;
        while (true) {
            boolean yet = false;
            for (VuforiaTrackable stone : skystones) {
                OpenGLMatrix position = ((VuforiaTrackableDefaultListener) stone.getListener()).getPose();

                if (position != null) {
                    yet = true;

                    VectorF translation = position.getTranslation();
                    float Xposition = translation.get(1);

                    String relativePosition = "Center";
                    if (Xposition < 10) {
                        relativePosition = "Left";
                        leftDrive.setPower(-0.25);
                        rightDrive.setPower(-0.25);
                    } else if (Xposition > 30){
                        leftDrive.setPower( 0.25);
                        rightDrive.setPower(0.25);
                    } else {
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                        sleep(100);
                    }

                    telemetry.addData("X: ", Xposition);
                    telemetry.update();

                    getout = true;
                }
            }

            if (!yet) {
                // Absolute angle
                double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                double error = currentAngle - AngleTarget;

                error = error * 0.05;

                leftDrive.setPower(-0.25 + error);
                rightDrive.setPower(-0.25 - error);
            }

            if (isStopRequested()) {
                break;
            }
        }

        CameraDevice.getInstance().setFlashTorchMode(false);

        leftDrive.setPower(0);
        rightDrive.setPower(0);
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

    // Mapeamento de Hardware
    // Ele utiliza as variáveis dispostas na área de "Componentes do Hardware"
    private void MapHardware() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        servo_0 = hardwareMap.get(Servo.class, "servo_0");
        servo_1 = hardwareMap.get(Servo.class, "servo_1");
        servo_2 = hardwareMap.get(Servo.class, "servo_2");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
    }

    // O robô só pode girar em dois sentidos: na esquerda e na direita
    enum DirectionSide {left, right}

    // O robô realiza um giro de 90° utilizando o IMU
    // É necessário realizar o mapeamento de hardware e inicializar o
    // IMU antes de utilizá-la
    private void RobotTurn90(DirectionSide side) {
        // Change angle target
        switch (side) {
            case left:
                AngleTarget += 90;
                break;
            case right:
                AngleTarget -= 90;
                break;
        }

        AngleTarget = AngleTarget % 180;

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + (side == DirectionSide.left ? -210 : 210));
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + (side == DirectionSide.left ? 210 : -210));

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(side == DirectionSide.left ? -ROTATION_VELOCITY : ROTATION_VELOCITY);    // 0.5
        rightDrive.setPower(side == DirectionSide.left ? ROTATION_VELOCITY : -ROTATION_VELOCITY);   // 0.5

        while(leftDrive.isBusy() && rightDrive.isBusy()){ }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // O robô anda em linha reta respeitando a variável AngleTarget
    // É necessário realizar o mapeamento de hardware e inicializar o
    // IMU antes de utilizá-la
    private void RobotDrive(double rotations, double velocity) {
        // Converts rotations to motor ticks
        rotations = rotations * 220;

        if (rotations < 0)
        {
            rotations *= -1;
            velocity *= -1;
        }

        double lastMotorL = leftDrive.getCurrentPosition();
        double lastMotorR = rightDrive.getCurrentPosition();
        while (Math.abs(leftDrive.getCurrentPosition()) < rotations + lastMotorL && Math.abs(rightDrive.getCurrentPosition()) < rotations + lastMotorR) {
            if (isStopRequested())
                break;

            // Absolute angle
            double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double error = currentAngle - AngleTarget;

            // Escala o erro
            error *= 0.05;

            leftDrive.setPower(velocity + error);   //(velocity + diff);
            rightDrive.setPower(velocity - error); //(velocity - diff);
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    // Os servos motor da garra só podem estar em dois estados:
    // aberto e fechado
    enum ServoConfiguration {open, close}

    // Os servos motor da garra são controlados com esta função
    // É necessário realizar o mapeamento de hardware antes de utilizá-la
    private void ServosPosition(ServoConfiguration config) {
        switch (config) {
            case open:
                servo_0.setPosition(0.5);
                servo_1.setPosition(-1);
                break;
            case close:
                servo_0.setPosition(-0.05);
                servo_1.setPosition(0.4);
                break;
        }
    }
}