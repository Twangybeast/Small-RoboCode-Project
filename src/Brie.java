import robocode.*;
import robocode.util.*;
import java.awt.Color;


// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * Brie - a robot by Smoo
 */
public class Brie extends AdvancedRobot
{
	/**
	 * run: Brie's default behavior
	 */
	
		private byte moveDirection = 1;
		
		

	public void run() {
		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:
		setAdjustGunForRobotTurn(true);
		setColors(Color.white,Color.yellow,Color.yellow); // body,gun,radar

		// Robot main loop
		while(true) {
			// Replace the next 4 lines with any behavior you would like

				turnGunRight(360);
				turnGunRight(360);
			
		}
	}
	
	public void radarLock(ScannedRobotEvent e){
		double radBearing = getHeadingRadians() + e.getBearingRadians();
		double turn = radBearing - getRadarHeadingRadians();
		setTurnRadarRightRadians(2.0*Utils.normalRelativeAngle(turn));
}

	public void aim(ScannedRobotEvent e){
		double radBearing = getHeadingRadians() + e.getBearingRadians();
		double turn = radBearing + Math.sin(e.getHeadingRadians()-radBearing)*Math.asin(e.getVelocity()/Rules.getBulletSpeed(2));
		setTurnGunRightRadians(Utils.normalRelativeAngle(turn - getGunHeadingRadians()));
}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// Replace the next line with any behavior you would like
		radarLock(e);
		move(e);
		aim(e);
		fire(2);
		/*
		 * int dist = e.getDistance;
		 * if
		 */
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Replace the next line with any behavior you would like
		moveDirection*= -1;
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		moveDirection*= -1;
	}	
	
	public void move(ScannedRobotEvent e){
		if (e.getDistance() > 300){
			setTurnRight(e.getBearing());
			setAhead(100);
		}else{
			setTurnRight(e.getBearing() + 65);
			setAhead(500 * moveDirection);
		}
		
	}
}
