package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class ChangekP extends Command {
    private Swerve s_Swerve;
    private double change;

    public ChangekP(Swerve s_Swerve, double change) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    public void execute() {
        s_Swerve.changePID(change);
    }

    public boolean isFinished() {
        return true;
    }
}