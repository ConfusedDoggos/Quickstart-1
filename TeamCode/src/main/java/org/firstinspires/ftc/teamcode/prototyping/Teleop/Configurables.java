package org.firstinspires.ftc.teamcode.prototyping.Teleop;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Configurables {
    public static double launcherTestSpeed = 0.6;

    public static boolean turretManualControl = false;

    public static double turretTolerance = 2;

    public static double tkSCustom = 0.13;

    public static double kp;
    public static double ki;
    public static double kd;
    public static double tkP;
    public static double tkI;
    public static double tkD;
    public static double errorTotal = 30;
}
