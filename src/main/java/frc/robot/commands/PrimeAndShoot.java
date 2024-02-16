package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class PrimeAndShoot extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;
    private final double targetSpeed;
    private Timer timer = new Timer();

    public PrimeAndShoot(Shooter s_Shooter, double targetSpeed) {
        this.s_Shooter = s_Shooter;
        this.targetSpeed = targetSpeed;
        addRequirements(s_Shooter);
    }
    
    public void execute() {
        s_Shooter.setPower(targetSpeed);
        timer.start();
        if (timer.get() >= 3){
            s_Intake.setPower(0.7);
        }

        /* TODO: run intake when shooter is at target speed */
    }
}
