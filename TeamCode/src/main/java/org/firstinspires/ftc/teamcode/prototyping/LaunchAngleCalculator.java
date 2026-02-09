package org.firstinspires.ftc.teamcode.prototyping;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchAngleCalculator {
    InterpLUT speedLUT = new InterpLUT();
    int y = 45 - 13;
    int g = 386;
    double minAngle = 35;
    double maxAngle = 60;
    public void createSpeedLUT() {
        speedLUT.add(0,0);
        speedLUT.add(2500,800);
        speedLUT.createLUT();
    }
    public double calcBestAngle(int launchVelocity,double x,double currentAngle) {

        double s = speedLUT.get(launchVelocity);

        double upperAngle = Math.toDegrees(Math.atan((s*s + Math.sqrt(s*s*s*s-g*(g*x*x+2*s*s*y)))/(g*x)));
        double lowerAngle = Math.toDegrees(Math.atan((s*s - Math.sqrt(s*s*s*s-g*(g*x*x+2*s*s*y)))/(g*x)));

        if (lowerAngle > minAngle && upperAngle < maxAngle) return lowerAngle;
        else if (upperAngle > minAngle && upperAngle < maxAngle) return upperAngle;
        else return currentAngle;

    }
}
