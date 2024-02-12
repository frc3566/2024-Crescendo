package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunIntakeWithTime extends Command {
    private Intake s_Intake;
    double power;
    double time;
    boolean done;
    private Timer timer = new Timer();

    public RunIntakeWithTime(Intake s_Intake, double power, double seconds) {
        this.s_Intake = s_Intake;
        this.power = power;
        this.time = seconds;
        addRequirements(s_Intake);
    }

    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }
    
    @Override
    public void execute() {
        s_Intake.setPower(power);
        if(timer.get() >= time) {
            System.out.println("cancelling");
            done = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.setPower(0);
    }

    public boolean isFinished() {
        return done;
    }
}
