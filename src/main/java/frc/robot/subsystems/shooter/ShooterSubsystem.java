package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.gos.lib.properties.GosDoubleProperty;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShooterSubsystem extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs;

  private final LoggedNetworkNumber shooterSpeed = new LoggedNetworkNumber("Shooter RPM", 3000);
  private final GosDoubleProperty dashboardShooterRPM =
      new GosDoubleProperty(false, "Shooter/Set RPM", 0.0);
  private final GosDoubleProperty dashboardHoodAngle =
      new GosDoubleProperty(false, "Shooter/Set Hood Angle", 0.0);

  public ShooterSubsystem(ShooterIO io) {
    m_io = io;
    m_inputs = new ShooterIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    Logger.recordOutput("Shooter/Shooter RPM", m_inputs.shooterSpeedL.in(RPM));
  }

  public Command setFlywheelVoltage(Voltage voltage) {
    return runEnd(
        () -> m_io.setShooterVoltage(voltage), () -> m_io.setShooterVoltage(Volts.of(0.0)));
  }

  public Command runShooterRPM(AngularVelocity rpm) {
    return runEnd(() -> m_io.setShooterRPM(rpm), () -> m_io.setShooterVoltage(Volts.of(0.0)));
  }

  public Command runDashboardRPM() {
    return runEnd(
        () -> m_io.setShooterRPM(RPM.of(dashboardShooterRPM.getValue())),
        () -> m_io.setShooterVoltage(Volts.of(0.0)));
  }

  public Command setHoodPosition(double position) {
    return runOnce(() -> m_io.setHoodDistance(position)).withName("Hood Position");
  }

  public Command setDashboardAngle() {
    return run(() -> m_io.setHoodDistance(dashboardHoodAngle.getValue()));
  }

  public Command runDasboard() {
    return runEnd(
            () -> {
              m_io.setShooterRPM(RPM.of(dashboardShooterRPM.getValue()));
              m_io.setHoodDistance(dashboardHoodAngle.getValue());
            },
            () -> {
              m_io.setShooterVoltage(Volts.of(0.0));
              m_io.setHoodDistance(0.1);
            })
        .withName("Shooter Auto Aim");
  }

  public Command autoShot() {
    return runEnd(
        () -> {
          m_io.setShooterRPM(RPM.of(RobotState.instance.getShooterRPM()));
          m_io.setHoodDistance(RobotState.getInstance().getHoodAngle());
        },
        () -> {
          m_io.setShooterVoltage(Volts.of(0.0));
          m_io.setHoodDistance(0.1);
        });
  }
}
