package frc.robot.Subsystems.Climber;

import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	RETRACTED("RETRACTED", ClimberConstants.RETRACT_SETPOINT),
	EXTEND("EXTEND", ClimberConstants.EXTEND_SETPOINT),
	HOLD("HOLD", ClimberConstants.HOLD_SETPOINT);

	private final String stateString;
	private final Double speed;

	ClimberStates(String stateString, double speed) {
		this.stateString = stateString;
		this.speed = speed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getClimberSpeed() {
		return speed;
	}
}
