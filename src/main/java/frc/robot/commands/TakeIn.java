package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class TakeIn extends Command{

    Shooter s_Shooter;
    Intake s_Intake;

    public final double intakeSpeed = 0.5;
    public final double shooterReverseSpeed = -0.3;
    
    public TakeIn(Shooter s_Shooter, Intake s_Intake){
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
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
        //TODO: Get sensors to detect the ring and return boolean
        return false;
    }
    
}
