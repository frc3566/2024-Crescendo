package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    // public DigitalInput sensor;
    public CANSparkMax intakeMotor;
    public double rtrigger;

    public Intake() {
        // sensor = new DigitalInput(0);
        intakeMotor = new CANSparkMax(Constants.Intake.Intake_Motor_Id, MotorType.kBrushless);
        intakeMotor.setInverted(false);
    }

    public void setPower(double power) {
        intakeMotor.set(power);
    }

    // public boolean getSensor() {
    //     return !sensor.get(); //Return 1 if object is closer than the set distance
    // }

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