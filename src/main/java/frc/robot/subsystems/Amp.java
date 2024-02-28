package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase{

    public CANSparkMax ampMotor;

    public Amp() {
        // sensor = new DigitalInput(0);
        ampMotor = new CANSparkMax(14, MotorType.kBrushless);
        ampMotor.setInverted(false);
        ampMotor.setSmartCurrentLimit(60);
    }

    public void setPower(double power) {
        ampMotor.set(power);
    }
    
    public void extend(){
        ampMotor.set(0.3);
    }

    public void retract(){
        ampMotor.set(-0.3);
    }

    public void stop() {
        ampMotor.stopMotor();
    }
}
