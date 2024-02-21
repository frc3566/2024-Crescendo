package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeTest extends Command {
    private Intake s_Intake;
    DoubleSupplier power;

    public IntakeTest(Intake s_Intake, DoubleSupplier power) {
        this.s_Intake = s_Intake;
        this.power = power;
        addRequirements(s_Intake);
    }
    
    @Override
    public void execute() {
        s_Intake.setPower(power.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stop();
    }
}
