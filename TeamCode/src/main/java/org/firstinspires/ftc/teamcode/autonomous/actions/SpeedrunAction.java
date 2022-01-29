package org.firstinspires.ftc.teamcode.autonomous.actions;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.autonomous.Constants;
import org.firstinspires.ftc.teamcode.autonomous.hardware.Hardware;
import org.firstinspires.ftc.teamcode.autonomous.localization.Localization;
import org.firstinspires.ftc.teamcode.autonomous.vision.Vuforia;


public class SpeedrunAction extends Action {

    public SpeedrunAction(int index){
        super(index);
    }

    public void execute(Hardware hardware, Localization localization, Vuforia vuforia){
        long time = System.currentTimeMillis();

        if (!Constants.IS_BLUE_TEAM) {
            hardware.setMotorValues(1, -1);
        }
        else{
            hardware.setMotorValues(-1, 1);
        }

        while(System.currentTimeMillis() - time < 3500)
        {
        }

        if (Constants.IS_BLUE_TEAM)
            while (Math.abs(hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Constants.BLUE_INITIAL_THETA) < 0.02)
            {
                hardware.setMotorValuesWithRotation(0, 0, 0.3);
            }
        else
            while (Math.abs(hardware.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - Constants.RED_INITIAL_THETA) < 0.02)
            {
                hardware.setMotorValuesWithRotation(0, 0, 0.3);
            }

                hardware.setMotorValues(0,0);
    }
}
