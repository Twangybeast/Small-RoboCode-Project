import robocode.*;
import java.awt.Color;

public class Kraken extends AdvancedRobot
{
	public void run() {
		setGunColor(Color.red);
		setBodyColor(Color.black);
		setBulletColor(Color.red);
		setRadarColor(Color.black);
		setScanColor(Color.black);
		setAdjustGunForRobotTurn(true);
		
		while(true) {
			setAhead(100);
			setTurnGunRight(360);
			setTurnLeft(10);
			execute();
		}
	}

	public void onScannedRobot(ScannedRobotEvent e) {
		if (getEnergy() > 50) {
			e.setPriority(98);
		}
		else {
			e.setPriority(90);
		}
		stop();
		if (e.getDistance() < 100) {
			fire(3);
		}
		else {
			fire(1.5);
		}
		if (getEnergy() > 50) {
			setTurnRight(e.getBearing());
			setTurnGunRight(360);
			setAhead(150);
			execute();
		}
		else {
			setTurnGunRight(360);
			setTurnLeft(10);
			setBack(100);
			execute();
		}
	}
	
	public void onHitByBullet(HitByBulletEvent e) {
		e.setPriority(95);
		setTurnGunRight(360);
		setTurnLeft(40);
		setAhead(50);
		execute();
	}
	
	public void onHitWall(HitWallEvent e) {
		e.setPriority(99);		
		setTurnGunRight(360);
		setBack(100);
		execute();
	}	
	
	public void onHitRobot(HitRobotEvent e) {
		e.setPriority(96);	
		if (getEnergy() < 50) {	
			setTurnGunRight(360);
			setTurnLeft(10);
			setBack(100);
			execute();
		}
	}	
}
