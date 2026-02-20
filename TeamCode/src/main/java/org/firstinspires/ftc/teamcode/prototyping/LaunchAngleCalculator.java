package org.firstinspires.ftc.teamcode.prototyping;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class LaunchAngleCalculator {
    InterpLUT speedLUT = new InterpLUT();

    InterpLUT angleLUT = new InterpLUT();
    //height is in meters
    int height = 3;
    double minHoodAngle = 35;
    double maxHoodAngle = 60;

    double closeDistance = .1;
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

        //didn't change much here, just made it easier to read (It does work). distance is meters and angles in radians
        double sqrt = Math.sqrt(Math.pow(velocity,4) - (9.8 * ((9.8 * Math.pow(distance, 2)) + (2 * height * Math.pow(velocity,2)))));
        double vel_1 = Math.pow(velocity,2);
        double divider = 9.8 * distance;
        
        double lowAngle = Math.atan((vel_1 - sqrt)/divider);
        double highAngle = Math.atan((vel_1 + sqrt)/divider);

        //converts hood angle into degrees and then into servo positions
        lowAngle = angleLUT.get(lowAngle * (180/Math.PI));
        highAngle = angleLUT.get(highAngle * (180/Math.PI));



        if (lowAngle > minHoodAngle && lowAngle < maxHoodAngle && highAngle > maxHoodAngle || highAngle < minHoodAngle) {
            return lowAngle;
        }
        else if (highAngle > minHoodAngle && highAngle < maxHoodAngle && lowAngle > maxHoodAngle || lowAngle < maxHoodAngle) {
            return highAngle;
        }
        else if (highAngle > maxHoodAngle || highAngle < minHoodAngle && lowAngle > minHoodAngle || lowAngle > maxHoodAngle) {
            if (distance <= closeDistance) {
                return highAngle;
            }
            else {
                return lowAngle;
            }
        }
        else {
            return -1;
        }
    }
}
