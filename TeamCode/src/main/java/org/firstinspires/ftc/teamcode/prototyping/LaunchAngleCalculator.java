package org.firstinspires.ftc.teamcode.prototyping;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchAngleCalculator {
    InterpLUT speedLUT = new InterpLUT();

    InterpLUT angleLUT = new InterpLUT();
    //height is in meters
    int height = 3;
    double minAngle = 35;
    double maxAngle = 60;
    private void createAngleLUT() {
        //for mapping angles (in degrees) to servo positions on the hood
        angleLUT.add(0,0);
        angleLUT.add(25, 60);
    }
    public void createSpeedLUT() {
        speedLUT.add(0,0);
        speedLUT.add(2500,800);
        speedLUT.createLUT();
    }
    public double calcBestAngle(double velocity, double distance) {

        //didnt change much here, just made it easier to read (It does work). distance is meters and angles in radians
        double sqrt = Math.sqrt(Math.pow(velocity,4) - (9.8 * ((9.8 * Math.pow(distance, 2)) + (2 * height * Math.pow(velocity,2)))));
        double vel_1 = Math.pow(velocity,2);
        double divider = 9.8 * distance;
        
        double minAngle = Math.atan((vel_1 - sqrt)/divider);
        double maxAngle = Math.atan((vel_1 + sqrt)/divider);

        if ((lowerAngle * 180/Math.PI) > minAngle && (upperAngle * 180/Math.PI) < maxAngle) return angleLUT.get(lowerAngle * 180/Math.PI);
        else if ((upperAngle * 180/Math.PI) > minAngle && (upperAngle * 180/Math.PI) < maxAngle) return angleLUT.get(upperAngle * 180/Math.PI);
        else return currentAngle;

    }
}
