package org.firstinspires.ftc.teamcode.autonomous.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TestOpMode extends OpMode {

    private DcMotor sliderMotor;
    private Servo boxServo;
    private DcMotor intakeMotor;
    private DcMotor leftfront, rightfront, leftback, rightback;

    @Override
    public void init() {
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMotor = hardwareMap.dcMotor.get("slideMotor");
        sliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        /*boxServo = hardwareMap.servo.get("boxServo");
        boxServo.setDirection(Servo.Direction.FORWARD);
         */
        leftfront = hardwareMap.dcMotor.get("leftFrontMotor");
        leftfront.setDirection(DcMotorSimple.Direction.FORWARD);


        rightfront = hardwareMap.dcMotor.get("rightFrontMotor");
        rightfront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftback = hardwareMap.dcMotor.get("leftBackMotor");
        leftback.setDirection(DcMotorSimple.Direction.FORWARD);

        rightback = hardwareMap.dcMotor.get("rightBackMotor");
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setPower(1);
    }

    @Override
    public void loop() {


        if (gamepad1.left_trigger > 0)
        {
            sliderMotor.setPower(gamepad1.left_trigger);
        } else
            sliderMotor.setPower(0.0000001);

        if (gamepad1.x)
            boxServo.setPosition(2/3);
        else
            if (gamepad1.b)
                boxServo.setPosition(0.000001);


    }
}
