package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeControl extends Command {

    private Intake s_Intake;
    private DoubleSupplier leftBumper, rightBumper;

    public IntakeControl(Intake s_Intake, DoubleSupplier leftBumper, DoubleSupplier rightBumper) {
        this.s_Intake = s_Intake;
        this.leftBumper = leftBumper;
        this.rightBumper = rightBumper;

        addRequirements(s_Intake);
    }
    
    @Override
    public void execute() {
        double power = 0;
        double lValue = leftBumper.getAsDouble();
        double rValue = rightBumper.getAsDouble();
        if ((lValue > 0 && rValue > 0) || (lValue == 0 && rValue == 0)){
            s_Intake.stop();
            return;
        }
        else if (lValue > 0){
            power = 1;
        }
        else{
            power = -1;
        }
        s_Intake.setPower(power);
    }
    
}
