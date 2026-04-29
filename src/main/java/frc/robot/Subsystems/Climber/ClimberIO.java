package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public interface ClimberIO {
	class ClimberIOOutputs {

		public Angle leftPosition = Rotations.of(0);
		public Angle rightPosition = Rotations.of(0);
		public double speed = 0;
	}

	void logOutputs(ClimberIOOutputs outputs);

	void setSetpoint(double setpoint);

}
