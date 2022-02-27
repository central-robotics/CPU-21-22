package org.firstinspires.ftc.teamcode.teleop.accessory;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.TeleOpHardware;
import java.util.concurrent.atomic.AtomicBoolean;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.teleop.DriveLoop;

public final class Intake {
    //private static final AtomicBoolean intakeThreadRunning = new AtomicBoolean(false);
    public static boolean intakeThreadRunning = false;
    public static boolean intakeRunning = false;
    public static double amin;
    public static double sam;

    public static void spinIntake(TeleOpHardware hardware, Gamepad gamepad)
    {
        intakeRunning = !intakeRunning;

        if (intakeRunning)
        {
            intakeThreadRunning = true;
            startThread(hardware, gamepad);
        }
        else {
            stopThread();
        }
    }

    private static void startThread(TeleOpHardware hardware, Gamepad gamepad)
    {

        new Thread(() ->
        {
            while (intakeThreadRunning == true) {
                hardware.intakeMotor.setPower(1);
                amin = hardware.intakeMotor.getCurrent(CurrentUnit.AMPS);
                if (Math.abs(hardware.intakeMotor.getCurrent(CurrentUnit.AMPS)) > 3) {
                    sam = 1;
                    while (hardware.intakeMotor.getCurrent(CurrentUnit.AMPS) > 1) {

                    }
                    sam = 2;
                    ElapsedTime time = new ElapsedTime();
                    time.reset();
                    while (time.milliseconds() < 1000){

                    }
                    sam = 3;
                    DriveLoop driveLoop = new DriveLoop();
                    driveLoop.moveSweeper(0.49, hardware);
                    time.reset();

                    while (time.milliseconds() < 2000){

                    }
                    sam = 4;
                    driveLoop.moveSweeper(1, hardware);


                    //hardware.intakeMotor.setPower(0.001);
                }


            }

            hardware.intakeMotor.setPower(0.001);

        }).start();

    }

    private static void stopThread()
    {
        intakeThreadRunning = false;
    }
}
