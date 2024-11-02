package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Constants;
public class LiftAngleMotor {

    private DcMotor liftanglemotor;

    public double location = liftanglemotor.getTargetPosition();

    public LiftAngleMotor(DcMotor liftanglemotor) {
        this.liftanglemotor = liftanglemotor;
    }

    public void LiftAngleMove(double speed, double location) {
        liftanglemotor.setTargetPosition(1);
        liftanglemotor.setPower(speed);
        liftanglemotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void LiftAngleFoward(double speed) {
        LiftAngleMove(speed, liftanglemotor.getCurrentPosition());
    }

    public void LiftAngleBackward(double speed) {
        LiftAngleMove(-speed, liftanglemotor.getCurrentPosition());
    }

    public void LiftAngleStop() {
        liftanglemotor.setPower(0);
    }
}
