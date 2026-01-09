package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Two Wheel Shooter RPM Test", group = "Examples")
public class TwoWheelShooterRPMTest extends OpMode {
    public int Motor1_currentRPM = 500;
    public int Motor2_currentRPM = 500;
    public DcMotorEx shooterMotorFront;
    public DcMotorEx shooterMotorBack;
    public Servo shooterLight;
    private Timer buttontimer;
    private  Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        shooterMotorFront = hardwareMap.get(DcMotorEx.class, "shooterMotorFront");
        shooterMotorBack = hardwareMap.get(DcMotorEx.class, "shooterMotorBack");
        shooterMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        buttontimer = new Timer();
        buttontimer.resetTimer();
    }

    @Override
    public void start() {
        buttontimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up && buttontimer.getElapsedTime() > 500) {
            Motor1_currentRPM += 25;
            shooterMotorFront.setVelocity(Motor1_currentRPM);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_down && buttontimer.getElapsedTime() > 500) {
            Motor1_currentRPM-= 25;
            shooterMotorFront.setVelocity(Motor1_currentRPM);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_right && buttontimer.getElapsedTime() > 500) {
            Motor2_currentRPM += 25;
            shooterMotorBack.setVelocity(Motor2_currentRPM);
            buttontimer.resetTimer();
        } else if (gamepad2.dpad_left && buttontimer.getElapsedTime() > 500) {
            Motor2_currentRPM -= 25;
            shooterMotorBack.setVelocity(Motor2_currentRPM);
            buttontimer.resetTimer();
        }

        if (gamepad2.b) {
            shooterMotorFront.setVelocity(robot.shooter.shooterCloseRPM);
            Motor1_currentRPM = robot.shooter.shooterCloseRPM;
        } else if (gamepad2.a) {
            shooterMotorBack.setVelocity(0);
            shooterMotorFront.setVelocity(0);
        } else if (gamepad2.y) {
            shooterMotorFront.setVelocity(robot.shooter.shooterMidRPM);
            Motor1_currentRPM = robot.shooter.shooterMidRPM;
        } else if (gamepad2.x) {
            shooterMotorFront.setVelocity(robot.shooter.shooterFarRPM);
            Motor1_currentRPM = robot.shooter.shooterFarRPM;
        }

//        if (currentRPM > 0 && shooterMotor.getVelocity() >= currentRPM) {
//            shooter.shooterLight.setPosition(0.5);
//        } else {
//            shooterLight.setPosition(0.3);
//        }

        if (gamepad2.right_trigger > 0.1) {
            robot.intake.startIntakeOnly();
        } else {
            robot.intake.stopIntake();
        }

        telemetry.addData("Motor1 Target RPM", Motor1_currentRPM);
        telemetry.addData("Motor1 Current Velocity", shooterMotorFront.getVelocity());
        telemetry.addData("Motor1 Motor Direction: ", shooterMotorFront.getDirection());
        telemetry.addData("Motor2 Target RPM", Motor2_currentRPM);
        telemetry.addData("Motor2 Current Velocity", shooterMotorBack.getVelocity());
        telemetry.addData("Motor2 Motor Direction: ", shooterMotorBack.getDirection());
        telemetry.update();
    }
}

