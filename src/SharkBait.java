/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import robocode.*;
import static robocode.util.Utils.normalRelativeAngleDegrees;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import java.awt.Color;
import robocode.util.*;

/**
 *
 * @author bartcha16
 */
public class SharkBait extends AdvancedRobot {
    boolean gunIdent;  
    boolean movingForward;
    boolean inWall;
    double xPos = 0;
    double yPos = 0;
    double xDif = 0;
    double speed = 10;
    double enemyHp;
    double damage = 2;
    double getTurn;
    static int HGShots;    
    static int LGShots;     
    static int HGHits;      
    static int LGHits;
    int moveDirection=1;
    Color getRandomColor[]=new Color[4];
    public void run() {
        setAdjustRadarForRobotTurn(true);
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        turnRadarRightRadians(Double.POSITIVE_INFINITY);
        do {
            scan();
                for(int i=0;i<4;i++)
                {
                    getRandomColor[i]=
                     Color.getHSBColor((float)Math.random(),(float)Math.random(),(float)Math.random());
                     }
                setColors(getRandomColor[0],getRandomColor[1],getRandomColor[2],Color.WHITE,getRandomColor[3]);
                setAdjustRadarForRobotTurn(true);
                setAdjustGunForRobotTurn(true);
                setAdjustRadarForGunTurn(true);
                 if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
                    inWall = true;
                } else {
                    inWall = false;
                }
                setAhead(40000);
                setTurnRadarRight(360);
                movingForward = true;
                while (true) {
                    if (getX() > 50 && getY() > 50 && getBattleFieldWidth() - getX() > 50 && getBattleFieldHeight() - getY() > 50 && inWall == true) {
                        inWall = false;
                    }
                    if (getX() <= 50 || getY() <= 50 || getBattleFieldWidth() - getX() <= 50 || getBattleFieldHeight() - getY() <= 50) {
                        if (inWall == false) {
                            reverseDirection();
                            inWall = true;
                        }
                    }
                    if (getRadarTurnRemaining() == 0.0) {
                        setTurnRadarRight(360);
                    }
                    execute();
                }
             }while (true);        
           }
    public void onScannedRobot(ScannedRobotEvent e) {
        double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
        setTurnRadarRightRadians(Utils.normalRelativeAngle(radarTurn));
        double firePower = Math.min(500 / e.getDistance(), 3);
        double bulletSpeed = 20 - firePower * 3;
        long time = (long) (e.getDistance() / bulletSpeed);
        damage = 10 - Math.round(e.getDistance() / 50);
        double gunTurnAmt;
        double absBearing=e.getBearingRadians()+getHeadingRadians();
        double latVel=e.getVelocity() * Math.sin(e.getHeadingRadians() -absBearing);
        if (e.getDistance() > 150) {
			gunTurnAmt = robocode.util.Utils.normalRelativeAngle(absBearing- getGunHeadingRadians()+latVel/22);
			setTurnGunRightRadians(gunTurnAmt);
			setTurnRightRadians(robocode.util.Utils.normalRelativeAngle(absBearing-getHeadingRadians()+latVel/getVelocity()));
			setAhead((e.getDistance() - 140)*moveDirection);
			setFire(3);
		}
	else{
			gunTurnAmt = robocode.util.Utils.normalRelativeAngle(absBearing- getGunHeadingRadians()+latVel/15);
			setTurnGunRightRadians(gunTurnAmt);
			setTurnLeft(-90-e.getBearing()); 
			setAhead((e.getDistance() - 140)*moveDirection);
			setFire(3);
		}
        if (damage < 1) {
            damage = 1;
        }
        double enemyX = Math.sin(Math.toRadians(getRadarHeading())) * e.getDistance() + getX();
        double enemyY = Math.cos(Math.toRadians(getRadarHeading())) * e.getDistance() + getY();
        double dirX = Math.sin(e.getHeadingRadians()) * e.getVelocity();
        double futureX = enemyX + (dirX * time);
        double calcX = (futureX - getX()) / e.getDistance();
        if (calcX < -1) {
            calcX = -1;
        }
        if (calcX > 1) {
            calcX = 1;
        }
        double getRotation = Math.asin(calcX);
        if (enemyY < getY()) {
            getRotation = -getRotation;
            setTurnGunRight(normalRelativeAngleDegrees(-getGunHeading() + Math.toDegrees(getRotation) - 180));
        } else {
            setTurnGunRight(normalRelativeAngleDegrees(-getGunHeading() + Math.toDegrees(getRotation)));
        }
        if (e.getDistance() < 500) {
            getTurn = getHeadingRadians() + e.getBearingRadians() - getHeadingRadians() + Math.PI * 0.5;
        } else {
            if (speed > 0) {
                getTurn = getHeadingRadians() + e.getBearingRadians() - getHeadingRadians() + Math.PI * 0.25;
            } else {
                getTurn = getHeadingRadians() + e.getBearingRadians() - getHeadingRadians() + Math.PI * 0.75;
            }
        }
        setTurnRightRadians(Utils.normalRelativeAngle(getTurn));
        if (getEnergy() < 40) {
            damage = 1;
        }
        if (e.getDistance() < 200) {
            damage = 10;
        }
        if (getEnergy() > 20 || e.getEnergy() < 1) {
            shoot();
        }
    }
    public void shoot() {
        fire(damage);
    }
    public void onHitWall(HitWallEvent e) {
       moveDirection=-moveDirection;
    }
     public void onBulletHit(BulletHitEvent e) {
		if(gunIdent) {
			LGHits = LGHits+1;
		} else {
			HGHits = HGHits+1;
		}
        }
    public void onWin(WinEvent e) {
        for (int i = 0; i < 256; i++) {
            Color wow = new Color(i, i, i);
            setColors(wow, wow, wow, wow, wow);
        }
    }

    public void reverseDirection() {
        if (movingForward) {
            setBack(40000);
            movingForward = false;
        } else {
            setAhead(40000);
            movingForward = true;
        }
    }
    public void onHitRobot(HitRobotEvent e) {
        if (e.isMyFault()) {
            reverseDirection();
        }
    }
}
