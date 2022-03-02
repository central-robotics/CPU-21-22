package org.firstinspires.ftc.teamcode.teleop.accessory;

import com.qualcomm.robotcore.hardware.DcMotor;
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
        if (!TeleOpConstants.isBlueOpMode)
            carouselSpinFactor.set(-1);
        else
            carouselSpinFactor.set(1);

        carouselThreadRunning.set(true);

        new Thread(() ->
        {
            double target = Math.abs(hardware.carouselMotor.getTargetPosition() + 4000);
            double current = Math.abs(hardware.carouselMotor.getCurrentPosition());

            while (current < target)
            {
                double error = current / target;

                hardware.carouselMotor.setPower(0.5 * carouselSpinFactor.get());
                current = Math.abs(hardware.carouselMotor.getCurrentPosition());

                if (!carouselThreadRunning.get())
                    break;
            }



            carouselThreadRunning.set(false);

            hardware.carouselMotor.setPower(0);

            hardware.carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }).start();

    }

    private static void stopThread()
    {
        carouselThreadRunning.set(false);
    }
}
