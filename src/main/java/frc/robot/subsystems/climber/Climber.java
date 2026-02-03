package frc.robot.subsystems.climber;

import static frc.robot.Constants.SubsystemConstants.CANivore;
import static frc.robot.Constants.SubsystemConstants.Climber.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor;
  private final DigitalInput limitSwitch;
  private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);

  public Climber() {
    climberMotor = new TalonFX(climberMotorID, CANivore);
    limitSwitch = new DigitalInput(limitSwitchPort);
    climberMotor.getConfigurator().apply(climberConfig);

    // Reset position to 0 when limit switch is triggered
    new Trigger(limitSwitch::get).onTrue(runOnce(() -> climberMotor.setPosition(0)));
  }

  public void setPosition(double targetPosition) {
    climberMotor.setControl(mmRequest.withPosition(targetPosition));
  }

  @AutoLogOutput(key = "Climber/Climber Zero'd?")
  public boolean getClimberZeroed() {
    return limitSwitch.get();
  }

  @AutoLogOutput(key = "Climber/Climber Position")
  public double getClimberPosition() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public Command togglePositionCommand() {
    return runOnce(
        () -> {
          double target =
              (Math.abs(getClimberPosition()) < climbUpHeight / 2.0) ? climbUpHeight : 0.0;
          setPosition(target);
        });
  }
}
