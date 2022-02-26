package org.firstinspires.ftc.teamcode.teleop.accessory;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.TeleOpHardware;
import java.util.concurrent.atomic.AtomicBoolean;

public final class Intake {
    private static final AtomicBoolean intakeThreadRunning = new AtomicBoolean(false);
    private static boolean intakeRunning = false;

    public static void spinIntake(TeleOpHardware hardware, Gamepad gamepad)
    {
        intakeRunning = !intakeRunning;

        if (intakeRunning)
        {
            startThread(hardware, gamepad);
        } else
            stopThread();
    }

    private static void startThread(TeleOpHardware hardware, Gamepad gamepad)
    {
        intakeThreadRunning.set(true);

        new Thread(() ->
        {
            hardware.intakeMotor.setPower(1);

            while (intakeThreadRunning.get())
            {
                if (hardware.intakeMotor.getCurrent(CurrentUnit.AMPS) > 4.85)
                    gamepad.rumble(500);
            }

            hardware.intakeMotor.setPower(0.001);
        }).start();

        intakeThreadRunning.set(false);

    }

    private static void stopThread()
    {
        intakeThreadRunning.set(false);
    }
}
