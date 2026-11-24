package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

import static edu.wpi.first.units.Units.*;

public class AutoCommands {
  public static Command lowerIntake(IntakeSubsystem intake) {
    return intake.setPivotPosition(IntakeSubsystem.Position.INTAKE);
  }

  public static Command runIntake(IntakeSubsystem intake) {
    return intake.setIntakePower(IntakeSubsystem.Speed.INTAKE);
  }

  public static Command stopIntake(IntakeSubsystem intake) {
    return intake.setIntakePower(IntakeSubsystem.Speed.STOP);
  }

  public static Command raiseIntake(IntakeSubsystem intake) {
    return intake.setPivotPosition(IntakeSubsystem.Position.STOWED);
  }

  public static Command aimAndShoot(
      Drive swerve, ShooterSubsystem shooter, FeederSubsystem feeder, IntakeSubsystem intake) {
    return DriveCommands.joystickDriveAtAngle(
            swerve,
            () -> 0.0,
            () -> 0.0,
            () ->
                RobotState.getInstance()
                    .getPointAtAngle(() -> AllianceFlipUtil.apply(FieldConstants.Hub.goalPoint)))
        .alongWith(shooter.autoShot())
                    .alongWith(
                            Commands.waitUntil(shooter::flywheelAtRPM)
                                    .andThen(feeder.runFeeder(Volts.of(12.0)))
                                    .alongWith(
                                            Commands.waitSeconds(1.0)
                                                    .andThen(intake.agitateCommand())
                                    )
                    ).withTimeout(6.0);
  }
}
