package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FeederIOTalonFX implements FeederIO {

  private final TalonFX m_feeder, m_column;
  private final StatusSignal<Voltage> feederVoltage, columnVoltage;
  private final StatusSignal<Current> feederStatorCurrent,
      feederSupplyCurrent,
      columnStatorCurrent,
      columnSupplyCurrent;

  private final VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);

  public FeederIOTalonFX() {
    m_feeder = new TalonFX(FeederConstants.FEEDER_ID);
    m_column = new TalonFX(FeederConstants.COLUMN_ID);

    m_feeder.getConfigurator().apply(FeederConstants.FEEDER_CONFIG);
    m_column
        .getConfigurator()
        .apply(FeederConstants.FEEDER_CONFIG.withMotorOutput(new MotorOutputConfigs()));

    feederVoltage = m_feeder.getMotorVoltage();
    feederStatorCurrent = m_feeder.getStatorCurrent();
    feederSupplyCurrent = m_feeder.getSupplyCurrent();

    columnVoltage = m_column.getMotorVoltage();
    columnStatorCurrent = m_column.getStatorCurrent();
    columnSupplyCurrent = m_column.getSupplyCurrent();

    // set refresh rates and clear all other signals from the CAN bus
    BaseStatusSignal.setUpdateFrequencyForAll(
        20,
        feederVoltage,
        feederSupplyCurrent,
        feederStatorCurrent,
        columnVoltage,
        columnSupplyCurrent,
        columnStatorCurrent);

    m_feeder.optimizeBusUtilization();
    m_column.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FeederIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        feederVoltage,
        feederSupplyCurrent,
        feederStatorCurrent,
        columnVoltage,
        columnSupplyCurrent,
        columnStatorCurrent);

    inputs.feederVoltage = feederVoltage.getValue();
    inputs.feederStatorCurrent = feederStatorCurrent.getValue();
    inputs.feederSupplyCurrent = feederSupplyCurrent.getValue();

    inputs.columnVoltage = columnVoltage.getValue();
    inputs.columnStatorCurrent = columnStatorCurrent.getValue();
    inputs.columnSupplyCurrent = columnSupplyCurrent.getValue();
  }

  @Override
  public void setFeederVoltage(Voltage voltage) {
    m_feeder.setControl(voltageOut.withOutput(voltage));
  }

  @Override
  public void setColumnVoltage(Voltage voltage) {
    m_column.setControl(voltageOut.withOutput(voltage));
  }
}
