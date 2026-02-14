package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExampleSubsystem extends SubsystemBase {

  public SparkMax motor1 = new SparkMax(1, MotorType.kBrushless);

  public ExampleSubsystem() {
    // motor1.setIdleMode(SparkMax.IdleMode.kBrake);
  }
}
