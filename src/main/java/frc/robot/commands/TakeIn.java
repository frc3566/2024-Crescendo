package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class TakeIn extends Command{

    Shooter s_Shooter;
    Intake s_Intake;
    Timer timer = new Timer();

    public final double intakeSpeed = 0.5;
    public final double shooterReverseSpeed = -0.3;
    
    public TakeIn(Shooter s_Shooter, Intake s_Intake){
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
    }

    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        s_Shooter.setPower(shooterReverseSpeed);
        s_Intake.setPower(intakeSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        s_Intake.stop();
    }

    public boolean isFinished(){
        return timer.get() >= 3;
    }
    
}
