package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunShooterWithTime extends Command {
    private Shooter s_Shooter;
    double power;
    double time;
    boolean done;
    private Timer timer = new Timer();

    public RunShooterWithTime(Shooter s_Shooter, double power, double seconds) {
        this.s_Shooter = s_Shooter;
        this.power = power;
        this.time = seconds;
        addRequirements(s_Shooter);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }
    
    @Override
    public void execute() {
        s_Shooter.setPower(power);
        if(timer.get() >= time) {
            System.out.println("cancelling");
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.setPower(0);
    }

    public boolean isFinished() {
        return done;
    }
}
