package org.firstinspires.ftc.teamcode.autonomous.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.autonomous.AutonCore;

public final class Motors extends AutonCore {

    public static DcMotor leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;

    public void InitializeMotors()
    {
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
