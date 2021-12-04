package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;

public abstract class Action {

    public final int index;

    public Action(int index)
    {
        this.index = index;
    }

    public abstract void execute(Hardware hardware, Localization localization);

}
