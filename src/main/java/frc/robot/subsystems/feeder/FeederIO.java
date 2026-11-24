package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  class FeederIOInputs {

    public Voltage feederVoltage = Volts.of(0.0);

    public Current feederStatorCurrent = Amps.of(0.0);

    public Current feederSupplyCurrent = Amps.of(0.0);

    public Voltage columnVoltage = Volts.of(0.0);

    public Current columnStatorCurrent = Amps.of(0.0);

    public Current columnSupplyCurrent = Amps.of(0.0);
  }

  public default void updateInputs(FeederIOInputsAutoLogged inputs) {}

  public default void setFeederVoltage(Voltage volts) {}

  public default void setColumnVoltage(Voltage volts) {}
}
