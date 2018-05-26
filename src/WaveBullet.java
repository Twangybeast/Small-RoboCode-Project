import robocode.util.Utils;

import java.awt.geom.Point2D;

public class WaveBullet
{
    private double startX, startY, startBearing, power;
    private long   fireTime;
    private int    direction;
    public double guessFactor = 0;

    public WaveBullet(double x, double y, double bearing, double power,
                      int direction, long time)
    {
        startX         = x;
        startY         = y;
        startBearing   = bearing;
        this.power     = power;
        this.direction = direction;
        fireTime       = time;
    }
    public void setPower(double power)
    {
        this.power = power;
    }

    public double getStartX()
    {
        return startX;
    }

    public double getStartY()
    {
        return startY;
    }

    public long getFireTime()
    {
        return fireTime;
    }

    public static double getBulletSpeed(double bulletPower)
    {
        return 20 - bulletPower* 3;
    }

    public static double maxEscapeAngle(double bulletPower)
    {
        return Math.asin(8 / getBulletSpeed(bulletPower));
    }
    public double getDistance(long currentTime)
    {
        return (currentTime - fireTime) * getBulletSpeed(power);
    }
    public double generateGuessFactor(double enemyX, double enemyY)
    {
        double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
        double angleOffset = Utils.normalRelativeAngle(desiredDirection - startBearing);
        return Math.max(-1, Math.min(1, angleOffset / maxEscapeAngle(power))) * direction;
    }
    public boolean checkHit(double enemyX, double enemyY, long currentTime)
    {
        // if the distance from the wave origin to our enemy has passed
        // the distance the bullet would have traveled...
        if (Point2D.distance(startX, startY, enemyX, enemyY) <= getDistance(currentTime))
        {
            guessFactor = generateGuessFactor(enemyX, enemyY);
            return true;
        }
        return false;
    }
}