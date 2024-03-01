package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.Timer;

public class PrimeAndShoot2 extends Command {
    private Shooter s_Shooter;
    private Intake s_Intake;

    private final double targetSpeed;

    private BooleanSupplier primeUntilTrue;

    private Timer timer = new Timer();
    private Timer intakeTimer = new Timer();

    public PrimeAndShoot2(Shooter s_Shooter, Intake s_Intake, double targetSpeed, BooleanSupplier primeUntilTrue) {
        this.s_Shooter = s_Shooter;
        this.s_Intake = s_Intake;
        this.targetSpeed = targetSpeed;
        this.primeUntilTrue = primeUntilTrue;
        addRequirements(s_Shooter, s_Intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakeTimer.reset();
    }
    
    @Override
    public void execute() {
        s_Shooter.setPower(targetSpeed);
        if (timer.get() >= 1 && primeUntilTrue.getAsBoolean() && intakeTimer.get() == 0) {
            intakeTimer.start();
            s_Intake.setPower(0.9);
        }
    }

    @Override
    public void end(boolean interrupted) {
        s_Shooter.stop();
        s_Intake.stop();
        timer.stop();
        intakeTimer.stop();
    }

    @Override 
    public boolean isFinished() {
        return intakeTimer.get() >= 1.5 && primeUntilTrue.getAsBoolean();
    }
}
