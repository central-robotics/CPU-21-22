package org.firstinspires.ftc.teamcode.teleop.accessory;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.shared.control.PID;
import org.firstinspires.ftc.teamcode.teleop.TeleOpConstants;
import org.firstinspires.ftc.teamcode.teleop.TeleOpHardware;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public final class Carousel {
    private static final AtomicBoolean carouselThreadRunning = new AtomicBoolean(false);
    private static final AtomicInteger carouselSpinFactor = new AtomicInteger(1);
    private static boolean carouselRunning = false;

    public static void spinCarousel(TeleOpHardware hardware)
    {
        carouselRunning = !carouselRunning;

        if (carouselRunning)
        {
            startThread(hardware);
        } else
            stopThread();
    }

    private static void startThread(TeleOpHardware hardware)
    {
        if (TeleOpConstants.isBlueOpMode)
            carouselSpinFactor.set(-1);
        else
            carouselSpinFactor.set(1);

        new Thread(() ->
        {
            PID carouselPID = new PID(new PIDCoefficients(0.001, 0, 0));
            double target = hardware.carouselMotor.getTargetPosition() + 4000;
            double current = hardware.carouselMotor.getCurrentPosition();

            while (current < target)
            {
                hardware.carouselMotor.setPower(carouselPID.getOutput(target - current, 0));
                current = hardware.carouselMotor.getCurrentPosition();

                if (!carouselThreadRunning.get())
                    break;
            }

            carouselThreadRunning.set(false);
        }).start();

    }

    private static void stopThread()
    {
        carouselThreadRunning.set(false);
    }
}
