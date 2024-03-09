// package frc.robot.autos;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.commands.intake.IntakeAndHold;
// import frc.robot.commands.shooter.PrimeAndShoot;
// import frc.robot.commands.shooter.PrimeWhileThenShoot;
// import frc.robot.commands.shooter.TeleopShoot;
// import frc.robot.commands.swerve.pid.*;
// import frc.robot.commands.vision.*;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.Vision;

// public class LeftSideTwoPiece extends SequentialCommandGroup {
//     private Pose2d targetPose = new Pose2d();
//     public LeftSideTwoPiece(Swerve s_Swerve, Intake s_Intake, Shooter s_Shooter, Vision s_Vision) {
//         // Command driveToFirstPosition = new Drive(s_Swerve, () -> new Pose2d());
//         // Command shootFirst = new PrimeAndShoot(s_Shooter, s_Intake, 1.0);
//         // Command driveWhileIntaking = new DriveAndIntake(s_Swerve, s_Intake, s_Shooter, new Pose2d(-2, 0, new Rotation2d()));
//         // Command lineUp = new DriveToAprilTag(s_Swerve, s_Vision);
//         // Command shootSecond = new PrimeAndShoot(s_Shooter, s_Intake, 1.0);
        
//         // addCommands(
//         //     driveToFirstPosition,
//         //     shootFirst,
//         //     driveWhileIntaking,
//         //     lineUp,
//         //     shootSecond
//         // );
//         targetPose = new Pose2d();
//         Drive moveBack = new Drive(s_Swerve, () -> new Pose2d(-1, 0, new Rotation2d(-45)));
//         Command moveBackAndShoot = moveBack.alongWith(new PrimeWhileThenShoot(s_Shooter, s_Intake, 1, () -> moveBack.isRunning()));
//         Command collectBackMiddleNote = new DriveAndIntake(s_Swerve, s_Intake, s_Shooter, new Pose2d(-3, 0, new Rotation2d(45)));
//         Command driveBack = new Drive(s_Swerve,() -> new Pose2d(3, 0, new Rotation2d(-45)));
//         AlignWithAprilTag alignToTag = new AlignWithAprilTag(s_Swerve, s_Vision);
//         Command lineUpAndShoot = alignToTag
//         .alongWith(new PrimeWhileThenShoot(s_Shooter, s_Intake, 1, () -> !alignToTag.isRunning()));
//         // Command getPose = new RadiusPose(s_Vision, new Pose2d(), (pose) -> targetPose = pose);

//     }
// }

// class DriveAndIntake extends SequentialCommandGroup {
//     public DriveAndIntake(Swerve s_Swerve, Intake s_Intake, Shooter s_Shooter, Pose2d pose) {
//         Command drive = new Drive(s_Swerve, () -> pose);
//         Command intake = new IntakeAndHold(s_Intake, s_Shooter, () -> true);

//         Command driveAndIntake = new ParallelDeadlineGroup(drive, intake);

//         addCommands(
//             driveAndIntake
//         );
//     }
// }