package org.firstinspires.ftc.teamcode.Meet_ILT;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class BallManager {
    private DistanceSensor distanceSensor_01;
    private double distance_01;

    public void init(HardwareMap hardwareMap) {
        distanceSensor_01 = hardwareMap.get(DistanceSensor.class, "firstDistanceSensor");
    }

    public boolean isBallInFirst() {
        distance_01 = distanceSensor_01.getDistance(DistanceUnit.INCH);
        if (distance_01 < 4) {
            return true;
        }
        else return false;
    }
}
