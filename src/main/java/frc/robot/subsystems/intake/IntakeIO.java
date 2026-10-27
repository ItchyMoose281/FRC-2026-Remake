package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public Voltage intakeVoltage = Volts.of(0.0);
    public Current intakeStatorCurrent = Amps.of(0.0);
    public Current intakeSupplyCurrent = Amps.of(0.0);

    public Angle pivotAngleDegrees = Degrees.of(0.0);
    public Voltage pivotVoltage = Volts.of(0.0);
    public Current pivotStatorCurrent = Amps.of(0.0);
    public Current pivotSupplyCurrent = Amps.of(0.0);
  }

  public default void updateInputs(IntakeIOInputsAutoLogged inputs) {}

  public default void setIntakeVoltage(Voltage voltage) {}

  public default void setPivotVoltage(Voltage voltage) {}

  public default void setPivotAngle(Angle angle) {}

  public default void resetPivotAngle(Angle angle) {}
}
