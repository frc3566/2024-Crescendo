package frc.robot.commands.intake;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTest extends Command {
    private Intake s_Intake;
    DoubleSupplier power;
    private double seconds;
    private Timer timer = new Timer();

    public IntakeTest(Intake s_Intake, DoubleSupplier power, double seconds) {
        this.s_Intake = s_Intake;
        this.power = power;
        this.seconds = seconds;
        addRequirements(s_Intake);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }
    
    @Override
    public void execute() {
        s_Intake.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stop();
    }

    @Override
    public boolean isFinished(){
        return timer.get() >= seconds;
    }
}
