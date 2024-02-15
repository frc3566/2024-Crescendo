package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPID extends SubsystemBase {

    public CANSparkMax left, right;
    public double ltrigger, rtrigger, currentSpeed, targetSpeed;
    public SparkPIDController PIDController;
    public ShooterPID() {
        left = new CANSparkMax(Constants.Shooter.Left_Motor_Id, MotorType.kBrushless);
        right = new CANSparkMax(Constants.Shooter.Right_Motor_Id, MotorType.kBrushless);
        left.setInverted(false);
        right.setInverted(true);
        left.setSmartCurrentLimit(20);
        right.setSmartCurrentLimit(20);



        PIDController = left.getPIDController();
        PIDController.setP(Constants.Shooter.ShooterKP);
        PIDController.setD(Constants.Shooter.ShooterKD);
        PIDController.setI(Constants.Shooter.ShooterKI);
    }
 
    public void setPower(double power) {
        left.set(power);
        right.set(power);
        PIDController.setReference(power, ControlType.kVelocity);
    }


    public void stop() {
        left.stopMotor();
        right.stopMotor();
    }
}