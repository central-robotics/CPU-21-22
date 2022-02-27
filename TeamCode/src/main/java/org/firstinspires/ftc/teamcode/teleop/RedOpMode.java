package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.teleop.accessory.Intake;

import java.io.File;

@TeleOp
public class RedOpMode extends OpMode {
    private TeleOpHardware hardware;
    private DriveLoop loop;

    @Override
    public void init() {
        File headingfile = AppUtil.getInstance().getSettingsFile("headingFile");
        String imu = ReadWriteFile.readFile(headingfile);
        TeleOpConstants.imuHeading = Double.parseDouble(imu);
        TeleOpConstants.isBlueOpMode = false;
        hardware = new TeleOpHardware();
        hardware.init(hardwareMap);
        loop = new DriveLoop();
    }

    @Override
    public void loop() {

        loop.loop(hardware, this);
    }

    public void stop()
    {
        Intake.intakeThreadRunning = false;
    }
}
