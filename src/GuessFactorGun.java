import java.awt.geom.*;
import java.util.ArrayList;
import java.util.List;

import com.sun.scenario.effect.impl.sw.sse.SSEBlend_SRC_OUTPeer;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

public class GuessFactorGun
{
    List<WaveBullet> waves = new ArrayList<WaveBullet>();
    static int[][] stats = new int[13][31];
    int direction = 1;
    public double gunAdjust;
    WaveBullet newWave;
    public void onScannedRobot(ScannedRobotEvent e, double headingRad, double x, double y, long time, double power, double gunHeading)
    {
        // ...
        // (other onScannedRobot code, might be radar/movement)
        // ...

        // Enemy absolute bearing, you can use your one if you already declare it.
        double absBearing = headingRad + e.getBearingRadians();

        // find our enemy's location:
        double ex = x + Math.sin(absBearing) * e.getDistance();
        double ey = y + Math.cos(absBearing) * e.getDistance();

        // Let's process the waves now:
        for (int i = 0; i < waves.size(); i++)
        {
            WaveBullet currentWave = (WaveBullet) waves.get(i);
            if (currentWave.checkHit(ex, ey, time))
            {
                waves.remove(currentWave);
                i--;
            }
        }
        // don't try to figure out the direction they're moving
        // they're not moving, just use the direction we had before
        if (e.getVelocity() != 0)
        {
            if (Math.sin(e.getHeadingRadians() - absBearing) * e.getVelocity() < 0)
                direction = -1;
            else
                direction = 1;
        }
        int[] currentStats = stats[(int)(e.getDistance() / 100)];
        // show something else later
        newWave = new WaveBullet(x, y, absBearing, power, direction, time, currentStats);
        int bestindex = 15;	// initialize it to be in the middle, guessfactor 0.
        for (int i=0; i<31; i++)
            if (currentStats[bestindex] < currentStats[i])
                bestindex = i;

        // this should do the opposite of the math in the WaveBullet:
        double guessfactor = (double)(bestindex - (currentStats.length - 1) / 2) / ((currentStats.length - 1) / 2);
        double angleOffset = direction * guessfactor * newWave.maxEscapeAngle();
        gunAdjust = Utils.normalRelativeAngle(absBearing - gunHeading + angleOffset);
        System.out.printf("%d\t\t%f\t\t%f\t\t%f\n", bestindex, guessfactor, angleOffset, gunAdjust);

    }
    public void postFire(boolean fired)
    {
        if (fired)
        {
            waves.add(newWave);
        }
    }
    public class WaveBullet
    {
        private double startX, startY, startBearing, power;
        private long   fireTime;
        private int    direction;
        private int[]  returnSegment;

        public WaveBullet(double x, double y, double bearing, double power,
                          int direction, long time, int[] segment)
        {
            startX         = x;
            startY         = y;
            startBearing   = bearing;
            this.power     = power;
            this.direction = direction;
            fireTime       = time;
            returnSegment  = segment;
        }
        public double getBulletSpeed()
        {
            return 20 - power * 3;
        }

        public double maxEscapeAngle()
        {
            return Math.asin(8 / getBulletSpeed());
        }
        public boolean checkHit(double enemyX, double enemyY, long currentTime)
        {
            // if the distance from the wave origin to our enemy has passed
            // the distance the bullet would have traveled...
            if (Point2D.distance(startX, startY, enemyX, enemyY) <=
                    (currentTime - fireTime) * getBulletSpeed())
            {
                double desiredDirection = Math.atan2(enemyX - startX, enemyY - startY);
                double angleOffset = Utils.normalRelativeAngle(desiredDirection - startBearing);
                double guessFactor =
                        Math.max(-1, Math.min(1, angleOffset / maxEscapeAngle())) * direction;
                int index = (int) Math.round((returnSegment.length - 1) /2 * (guessFactor + 1));
                returnSegment[index]++;
                return true;
            }
            return false;
        }
    }
}
