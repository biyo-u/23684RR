package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Constants;
public class LiftAngleMotor {

    private final DcMotor liftanglemotor;

    public LiftAngleMotor(DcMotor liftanglemotor) {
        this.liftanglemotor = liftanglemotor;
    }

    public void LiftAngleMove(double speed, double location) {
        liftanglemotor.setTargetPosition(1);
        liftanglemotor.setPower(speed);
        
    }
}
