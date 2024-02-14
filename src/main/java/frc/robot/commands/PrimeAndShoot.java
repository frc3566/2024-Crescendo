package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class PrimeAndShoot extends Command {
    private Shooter s_Shooter;
    private final double targetSpeed;

    public PrimeAndShoot(Shooter s_Shooter, double targetSpeed) {
        this.s_Shooter = s_Shooter;
        this.targetSpeed = targetSpeed;
        addRequirements(s_Shooter);
    }
    
    public void execute() {
        s_Shooter.setPower(targetSpeed);

        /* TODO: run intake when shooter is at target speed */
    }
}
