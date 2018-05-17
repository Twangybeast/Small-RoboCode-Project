import robocode.*;
import java.awt.Color;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * Test - a robot by (your name here)
 */
public class Test extends AdvancedRobot{
int wallMargin = 50;
  	int movementDirection = 1;
	double previousEnergy = 100;
	public void run() {
		setRadarColor(new Color(60, 212, 187)); //teal
		setScanColor(new Color(140, 198, 188));
		setBodyColor(new Color(36, 108, 96));
		setGunColor(new Color(60, 212, 187));
		setBulletColor(new Color(60, 212, 187));
		
		/*setRadarColor(new Color(247, 141, 191)); //pink
		setScanColor(new Color(247, 141, 191));
		setBodyColor(new Color(236, 98, 163));
		setGunColor(new Color(247, 141, 191));
		setBulletColor(new Color(247, 141, 191));*/   
		
		/*while(true){ //goes around other robot
			setAdjustGunForRobotTurn(true);
			turnGunRight(360);
			setAhead(100);
			turnRight(360);
		}*/

	
		while (true) {
			ahead(100 * movementDirection); 
			turnGunRight(360);
			//turnRadarRight(360);
			back(100 * movementDirection); 
			turnGunRight(360);
			//turnRadarRight(360);
			//scan();
		}
	}

	
	public void onScannedRobot(ScannedRobotEvent e) {
		turnGunRight(getHeading() - getGunHeading() + e.getBearing());
		fire(1);
		
	    double distanceFromEnemy = e.getDistance();
		if(distanceFromEnemy < 50) {
			turnRight(e.getBearing() + 130);
			ahead(100);
		}
		
		/*double enemyEnergy = e.getEnergy();
		if(enemyEnergy < 20) {
			turnGunRight(getHeading() - getGunHeading() + e.getBearing());
			fire(2);
		}*/
		
		double changeInEnergy = previousEnergy - e.getEnergy();
		if(changeInEnergy > 0 && changeInEnergy < 5){
			movementDirection = -movementDirection;
		}
		

	    if(getX() <= wallMargin){
			turnRight(e.getBearing() + 180);
			ahead(100);
		} 
		if(getX() >= getBattleFieldWidth() - wallMargin){
			turnRight(e.getBearing() + 180);
			ahead(100);
		}
		if(getY() <= wallMargin) {
			turnRight(e.getBearing() + 180);
			ahead(100);
		}
		if(getY() >= getBattleFieldHeight() - wallMargin) {
			turnRight(e.getBearing() + 180);
			ahead(100);
		}
		scan();
	}


	public void onHitByBullet(HitByBulletEvent e) {
		//turnLeft(90 - e.getBearing());
		turnRight(e.getBearing() + 75);
		ahead(150);
		
	}
	
	public void onHitWall(HitWallEvent e) {
		turnRight(e.getBearing() + 180);
		ahead(100);
		//moveDirection *= -1;
	}	
}