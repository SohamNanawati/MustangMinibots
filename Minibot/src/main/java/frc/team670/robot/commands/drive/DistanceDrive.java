package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.Robot;
import frc.team670.robot.RobotConstants;
import frc.team670.robot.RobotContainer;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;

public class DistanceDrive extends CommandBase {
	
	private double speedL, speedR, dist;
	private final double LEFT_ENCODER_NUM;
	private boolean correcting=false;
	

	private DriveBase driveBase;
	
	/**
	 * 
	 * @param distance_in Target distance in inches
	 * @param lspeed Speed for left side
	 * @param rspeed Speed for right side
	 */
	public DistanceDrive(double distance_in, double lspeed, double rspeed, DriveBase driveBase) {
		this.speedL = lspeed;
		this.speedR = rspeed;
		this.LEFT_ENCODER_NUM = 1.26421;
		//this.seconds = seconds;
		this.dist = distance_in;
		this.driveBase = driveBase;
		addRequirements(driveBase);
	}

	// Called just before this Command runs the first time
	
	public void initialize() {
		//setTimeout(seconds);
		Logger.consoleLog("LeftSpeed: %s Right Speed: %s DistanceT: %s", 
				speedL, speedR, getDistance());
	}	

	// Called repeatedly when this Command is scheduled to run
	
	public void execute() { 
		Logger.consoleLog("LeftSpeed: %s Right Speed: %s DistanceT: %s TicksL: %s TicksR %s", 
				speedL, speedR, getDistance(), (driveBase.getLeftEncoder().getTicks()/LEFT_ENCODER_NUM),-driveBase.getRightEncoder().getTicks());
				//speedL, speedR, getDistance(), (driveBase.getLeftEncoder().getTicks()),-driveBase.getRightEncoder().getTicks());		
		
		correcting = false;
		if(!correcting){
			driveBase.tankDrive(speedL, speedR);
		}
		correct();

	
	}

	
	// Called once after isFinished returns true
	
	public void end(boolean interrupted) {
		driveBase.stop();
		Logger.consoleLog("LeftSpeed: %s Right Speed: %s DistanceT: %s Ticks: %s", 
				speedL, speedR, getDistance(), Math.abs(driveBase.getRightEncoder().getTicks()));
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	public void interrupted() {
		end(true);
	}	
	
	// Checks that the wheels are driving at the same speed, corrects the speed
	// so that the left/right are equal
	public void correct() {
		double currentTicksL = driveBase.getLeftEncoder().getTicks()/LEFT_ENCODER_NUM;
		//double currentTicksL = driveBase.getLeftEncoder().getTicks();
		double currentTicksR = -driveBase.getRightEncoder().getTicks();
		
		if (Math.abs(currentTicksL - currentTicksR) < 5)
			return;
		
		//else if (currentTicksL > currentTicksR)
		// 		speedL -= 0.05;
		
		//else if (currentTicksL < currentTicksR)
		// 		speedR -= 0.05;
		else if (currentTicksL < currentTicksR){
			speedR -= 0.015;
			speedL += 0.015;
		}
		else if (currentTicksL > currentTicksR){
			speedL -= 0.015;
			speedR += 0.015;
		}
		speedL = (speedL > 1) ? 1 : speedL;
		speedL= (speedL < -1) ? -1 : speedL;

		speedR = (speedR > 1) ? 1 : speedR;
		speedR = (speedR < -1) ? -1: speedR;
			
	}		
	
	// Make this return true when this Command no longer needs to run execute()
		@Override
		public boolean isFinished() {
			// if(getDistance() > Math.abs(dist)){
			// 	double currentTicksL = driveBase.getLeftEncoder().getTicks()/LEFT_ENCODER_NUM;
			// 	//double currentTicksL = driveBase.getLeftEncoder().getTicks();
			// 	double currentTicksR = -driveBase.getRightEncoder().getTicks();
			// 	if (Math.abs(currentTicksL - currentTicksR) < 10) {
			// 		return true;
			// 	}
			// 	if (currentTicksL > currentTicksR){
			// 		driveBase.tankDrive(0, 1);
			// 	}else{
			// 		driveBase.tankDrive(1, 0);

			// 	}
			// 	correcting=true;
			// 	return false;
			// }
			return getDistance() >= Math.abs(dist);
			//return (this.error <= 1);
		}
		
			
		public double getDistance()
		{
			double distance = driveBase.getRightEncoder().getDistance();
			return Math.abs(distance);
		}

}

	

