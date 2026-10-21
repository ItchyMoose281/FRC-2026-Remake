package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO{
//    private final TalonFX m_pivot, m_intake;
    private final MotionMagicVoltage m_mmVoltage = new MotionMagicVoltage(0.0);


}
