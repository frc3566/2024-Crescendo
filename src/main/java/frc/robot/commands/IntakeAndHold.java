package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

public class IntakeAndHold extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;

    private final double intakeSpeed = 0.5;
    private final double shooterReverseSpeed = -0.3;
    private final double intakeReverseSpeed = -0.1;

    public IntakeAndHold(Intake s_Intake, Shooter s_Shooter) {
        this.s_Intake = s_Intake;
        this.s_Shooter = s_Shooter;
        addRequirements(s_Intake, s_Shooter);
    }
    
    @Override
    public void execute() {
        s_Intake.setPower(intakeSpeed);
        s_Shooter.setPower(shooterReverseSpeed);

        /* if (sensor detects note) {
            end(false);
        } */

        if(!s_Intake.getSensor()){
            end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {

        // if(!interrupted){
        //     if(s_Intake.getSensor()){ 
        //         s_Intake.setPower(intakeReverseSpeed);
        //     }
        // }

        s_Intake.stop();
        s_Shooter.stop();

        /* if (!interrupted) {
            make sure note is in intake and not touching shooter flywheels
        } */
        
        
    }
}
