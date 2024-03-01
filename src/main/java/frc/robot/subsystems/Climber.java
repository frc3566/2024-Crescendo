package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber {
    public CANSparkMax leftClimber;
    public CANSparkMax rightClimber;


    public Climber(){
        leftClimber = new CANSparkMax(Constants.Climber.Left_Climber_Id, MotorType.kBrushless);
        rightClimber = new CANSparkMax(Constants.Climber.Right_Climber_Id, MotorType.kBrushless);
        leftClimber.setInverted(false);
        rightClimber.setInverted(false);
        leftClimber.setSmartCurrentLimit(40);
        rightClimber.setSmartCurrentLimit(40);
    }

    public void setPower(double power) {
        leftClimber.set(power);
        rightClimber.set(power);
    }

    public void leftclimb(){
        leftClimber.set(1);
    }

    public void rightclimb(){
        rightClimber.set(1);
    }

    public void leftretract(){
        leftClimber.set(-1);
    }

    public void rightretract(){
        rightClimber.set(-1);
    }

    public void stop() {
        leftClimber.stopMotor();
        rightClimber.stopMotor();
    }
}
