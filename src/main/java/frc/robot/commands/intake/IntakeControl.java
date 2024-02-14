package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeControl extends Command {

    private Intake s_Intake;
    private DoubleSupplier lTrigger, rTrigger;

    public IntakeControl(Intake s_Intake, DoubleSupplier lTrigger, DoubleSupplier rTrigger) {
        this.s_Intake = s_Intake;
        this.lTrigger = lTrigger;
        this.rTrigger = rTrigger;

        addRequirements(s_Intake);
    }
    
    @Override
    public void execute() {
        double lValue = lTrigger.getAsDouble();
        double rValue = rTrigger.getAsDouble();
        if ((lValue > 0 && rValue > 0) || (lValue == 0 && rValue == 0)){
            s_Intake.stop();
            return;
        }
        double power = Math.max(lValue, rValue) * ( lValue > rValue ? -1 : 1) * -0.5;
        s_Intake.setPower(power);
    }
    
}
