package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    public CANSparkMax intakeMotor;
    public double rTrigger;

    public Intake() {
        intakeMotor = new CANSparkMax(Constants.Intake.Intake_Motor_Id, MotorType.kBrushless);
        intakeMotor.setInverted(false);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
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
        intakeMotor.stopMotor();
    }
}