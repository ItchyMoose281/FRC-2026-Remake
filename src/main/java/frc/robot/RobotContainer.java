// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.*;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive swerve;
  private final IntakeSubsystem intake;
  private final FeederSubsystem feeder;
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

  // Dashboard inputs
  private final SendableChooser<Command> compAutoChooser;
  private final SendableChooser<Command> testAutoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        swerve =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new IntakeSubsystem(new IntakeIOTalonFX());
        feeder = new FeederSubsystem(new FeederIOTalonFX());
        shooter = new ShooterSubsystem(new ShooterIOTalonFX());
        vision =
            new VisionSubsystem(
                VisionConstants.FILTER_PARAMETERS,
                new VisionIOPhotonReal(
                    "FrontCamera",
                    VisionConstants.FRONT_CAMERA_TRANSFORM,
                    FieldConstants.defaultAprilTagType.getLayout()),
                new VisionIOPhotonReal(
                    "LeftCamera",
                    VisionConstants.LEFT_CAMERA_TRANSFORM,
                    FieldConstants.defaultAprilTagType.getLayout()),
                new VisionIOPhotonReal(
                    "RightCamera",
                    VisionConstants.RIGHT_CAMERA_TRANSFORM,
                    FieldConstants.defaultAprilTagType.getLayout()));
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        swerve =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new IntakeSubsystem(new IntakeIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        vision =
            new VisionSubsystem(
                VisionConstants.FILTER_PARAMETERS,
                new VisionIOPhotonSimulation(
                    "LeftCamera",
                    VisionConstants.LEFT_CAMERA_TRANSFORM,
                    FieldConstants.defaultAprilTagType.getLayout(),
                    VisionConstants.SIM_CAMERA_PROPERTIES));
      }

      case REPLAY -> {
        swerve =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new IntakeSubsystem(new IntakeIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        shooter = new ShooterSubsystem(new ShooterIO() {});
        vision =
            new VisionSubsystem(
                VisionConstants.FILTER_PARAMETERS,
                new VisionIOPhotonSimulation(
                    "LeftCamera",
                    VisionConstants.LEFT_CAMERA_TRANSFORM,
                    FieldConstants.defaultAprilTagType.getLayout(),
                    VisionConstants.SIM_CAMERA_PROPERTIES));
      }
      default -> throw new IllegalStateException("Unexpected value: " + Constants.getMode());
    }

      compAutoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
              "", // Defaults to "None" (Commands.none())
              (stream) -> stream.filter(auto -> !auto.getName().toLowerCase().startsWith("test"))
      );

      // 2. Test Chooser: includes ONLY autos starting with "test"
      testAutoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
              "", // Defaults to "None" (Commands.none())
              (stream) -> stream.filter(auto -> auto.getName().toLowerCase().startsWith("test"))
      );

    // Set up SysId routines
    testAutoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(swerve));
    testAutoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(swerve));
    testAutoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    testAutoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    testAutoChooser.addOption(
        "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    testAutoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    swerve.setDefaultCommand(
        DriveCommands.joystickDrive(
            swerve,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX()));

    vision.setDefaultCommand(
        vision.processVision(RobotState.getInstance()::getEstimatedPose).ignoringDisable(true));

    // Start button: reset robot pose to origin
    driveController.start().onTrue(Commands.runOnce(() -> swerve.setPose(new Pose2d())));

    // B button: run feeder at 12V
    driveController.b().whileTrue(feeder.runFeeder(Volts.of(12.0)));

    // Y button: agitate intake
    driveController.y().whileTrue(intake.agitateCommand());

    // Left stick button: home the intake
    driveController.leftStick().onTrue(intake.homingCommand());

    // Right bumper: deploy and run intake
    driveController.rightBumper().whileTrue(intake.intake());

    // Left bumper: stow intake
    driveController.leftBumper().onTrue(intake.setPivotPosition(IntakeSubsystem.Position.STOWED));

    // Right Trigger: run shooter as set on dashboard
    driveController.leftTrigger().whileTrue(shooter.runDasboard());

    driveController
        .rightTrigger()
        .and(driveController.leftTrigger().negate())
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    swerve,
                    () -> -driveController.getLeftY(),
                    () -> -driveController.getLeftX(),
                    () ->
                        RobotState.getInstance()
                            .getPointAtAngle(
                                () -> AllianceFlipUtil.apply(FieldConstants.Hub.goalPoint)))
                .alongWith(shooter.autoShot()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      Command compAuto = compAutoChooser.getSelected();

      // Prioritize Comp Auto whenever one is selected (i.e., not Commands.none())
      if (compAuto != null && !compAuto.equals(Commands.none())) {
          return compAuto;
      }

      // Hard safety lock: block test autos whenever connected to the FMS
      if (DriverStation.isFMSAttached()) {
          return Commands.none();
      }

      return testAutoChooser.getSelected();
  }
}
