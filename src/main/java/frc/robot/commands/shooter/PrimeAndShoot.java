package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class PrimeAndShoot extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;
    private final double targetSpeed;
    private Timer timer;

    public PrimeAndShoot(Shooter s_Shooter, double targetSpeed) {
        this.s_Shooter = s_Shooter;
        this.targetSpeed = targetSpeed;
        this.timer = new Timer();

        addRequirements(s_Shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        s_Shooter.setPower(targetSpeed);
        if (timer.get() >= 1) {
            s_Intake.setPower(0.9);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        s_Intake.stop();
        timer.stop();
    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= 2.5;
    }
}
