package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public AngularVelocity shooterSpeedL, shooterSpeedM, shooterSpeedR = RPM.of(0.0);
    public Voltage shooterVoltageL, shooterVoltageM, shooterVoltageR = Volts.of(0.0);
    public Current shooterStatorCurrentL,
        shooterStatorCurrentM,
        shooterStatorCurrentR = Amps.of(0.0);
    public Current shooterSupplyCurrentL,
        shooterSupplyCurrentM,
        shooterSupplyCurrentR = Amps.of(0.0);

    public double[] hoodPositionPercent = {0.0, 0.0};
  }

  public default void updateInputs(ShooterIOInputsAutoLogged inputs) {}

  default void setShooterVoltage(Voltage voltage) {}

  default void setShooterRPM(AngularVelocity velocity) {}

  default void setHoodDistance(double position) {}
}
