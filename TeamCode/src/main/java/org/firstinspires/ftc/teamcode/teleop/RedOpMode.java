package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RedOpMode extends OpMode {
    private TeleOpHardware hardware;
    private DriveLoop loop;

    @Override
    public void init() {
        TeleOpConstants.isBlueOpMode = false;
        hardware = new TeleOpHardware();
        hardware.init(hardwareMap);
        loop = new DriveLoop();
    }

    @Override
    public void loop() {

        loop.loop(hardware, this);
    }
}
