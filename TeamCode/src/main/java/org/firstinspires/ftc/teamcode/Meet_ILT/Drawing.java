package org.firstinspires.ftc.teamcode.Meet_ILT;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;

public class Drawing {
    FieldManager panelsField = PanelsField.INSTANCE.getField();

    void initilize() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }
    void updatePose(double x, double y) {
        panelsField.setCursorX(x);
        panelsField.setCursorY(y);
        panelsField.circle(10);

        panelsField.update();
    }
}
