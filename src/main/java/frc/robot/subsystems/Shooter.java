package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public CANSparkMax left, right;
    public double ltrigger, rtrigger;

    public Shooter() {
        left = new CANSparkMax(Constants.Shooter.Left_Motor_Id, MotorType.kBrushless);
        right = new CANSparkMax(Constants.Shooter.Right_Motor_Id, MotorType.kBrushless);
        left.setInverted(false);
        right.setInverted(true);
    }

    public void setPower(double power) {
        left.set(power);
        right.set(power);
    }

    // public void setVoltage(double voltage) {
    //     left.setVoltage(voltage);
    //     right.setVoltage(voltage);
    // }

    // public void setBrake(boolean isBrake) {
    //     IdleMode sparkMode = isBrake ? IdleMode.kBrake : IdleMode.kCoast;
    //     left.setIdleMode(sparkMode);
    //     right.setIdleMode(sparkMode);
    // }

    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }
}