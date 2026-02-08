package org.firstinspires.ftc.teamcode.Meet_ILT;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Drawing {

    private static final Style robotLook = new Style(
            "", "#4CAF50", 0.75
    );
    FieldManager panelsField = PanelsField.INSTANCE.getField();

    void initilize() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }
    void drawRobot(Position pose, YawPitchRollAngles rollAngles) {
        panelsField.setStyle(robotLook);
        panelsField.moveCursor(pose.x, pose.y);
        panelsField.circle(10);

        Vector v = new Vector(1, rollAngles.getYaw(AngleUnit.RADIANS));
        v.setMagnitude(v.getMagnitude() * 10);
        double x1 = pose.x + v.getXComponent() / 2, y1 = pose.y + v.getYComponent() / 2;
        double x2 = pose.x + v.getXComponent(), y2 = pose.y + v.getYComponent();

        panelsField.setStyle(robotLook);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);

        panelsField.update();
    }
}
