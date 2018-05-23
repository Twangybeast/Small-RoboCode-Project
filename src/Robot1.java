import robocode.AdvancedRobot;
import robocode.HitByBulletEvent;
import robocode.Rules;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class Robot1 extends AdvancedRobot
{
    double oldEnemyHeading = -1;
    double oldEnemyVelocity = 0;
    public static int BINS = 47;
    public static double _surfStats[] = new double[BINS];
    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _enemyLocation;  // enemy bot's location

    public ArrayList _enemyWaves;
    public ArrayList _surfDirections;
    public ArrayList _surfAbsBearings;

    public static double  _oppEnergy = 100.0;
    public static Rectangle2D.Double _fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public static double WALL_STICK = 160;

    //Info: Enemy heading, heading change, distance, velocity, acceleration,
    public static int DIMENSIONS = 5;
    public static KDTree<KDPayload> history = new KDTree.WeightedManhattan<>(DIMENSIONS);
    public LinkedList<DNNNode> shortHistory = new LinkedList<>();
    public static int MIN_HISTORY_TO_FIRE = 30;
    public static final int NEIGHBOR_COUNT = 100;

    GuessFactorGun gun = new GuessFactorGun();
    //TODO
    //Insert to KDTREE after wave breaks
    //Have symmetry w/ +/- velocity
    //Fix nearest bearing
    @Override
    public void run()
    {
        ((KDTree.WeightedManhattan)history).setWeights(new double[]{1, 1, 2, 1, 1});
        setAllColors(Color.YELLOW);
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        _enemyWaves = new ArrayList();
        _surfDirections = new ArrayList();
        _surfAbsBearings = new ArrayList();
        do{
            if(getRadarTurnRemainingRadians() == 0){
                setTurnRadarRight(Double.POSITIVE_INFINITY);
            }
            execute();
        }while(true);
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e)
    {
        _myLocation = new Point2D.Double(getX(), getY());
        double lateralVelocity = getVelocity() * Math.sin(e.getBearingRadians());


        //double bulletPower = Math.max(Math.min(1.95, e.getEnergy()/4), 3.0);
        double bulletPower = Math.min(Math.min(1.95, 1.95), 3.0);
        if (getEnergy() < 50.0 )
        {
            bulletPower = 0;
        }



        double myX = getX();
        double myY = getY();
        double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyHeading = e.getHeadingRadians();
        if (oldEnemyHeading == -1)
        {
            oldEnemyHeading = enemyHeading;
        }
        double enemyHeadingChange = enemyHeading - oldEnemyHeading;
        double enemyVelocity = e.getVelocity();
        oldEnemyHeading = enemyHeading;
        double enemyVelocityChange = enemyVelocity - oldEnemyVelocity;


        //Surfing
        _surfDirections.add(0, new Integer((lateralVelocity >= 0) ? 1 : -1));
        _surfAbsBearings.add(0, new Double(absoluteBearing + Math.PI));

        //Recording positions
        double[] positionInfo = new double[]{
                Utils.normalRelativeAngle(e.getHeadingRadians() - absoluteBearing)/(Math.PI*2),
                Utils.normalRelativeAngle(enemyHeadingChange)/(Math.PI * 2 / 36),
                e.getDistance()/1000,
                enemyVelocity/8,
                enemyVelocityChange/2
        };
        int direction = 1;
        if (e.getVelocity() != 0)
        {
            if (Math.sin(e.getHeadingRadians() - absoluteBearing) * e.getVelocity() < 0)
                direction = -1;
            else
                direction = 1;
        }
        DNNNode currentNode = new DNNNode(positionInfo, new WaveBullet(getX(), getY(), absoluteBearing, bulletPower, direction, getTime()));
        shortHistory.add(currentNode);
        double enemyX = getX() + e.getDistance() * Math.sin(absoluteBearing);
        double enemyY = getY() + e.getDistance() * Math.cos(absoluteBearing);

        for (Iterator<DNNNode> iter = shortHistory.iterator();iter.hasNext();)
        {
            DNNNode node = iter.next();
            if (node.waveBullet.checkHit(enemyX, enemyY, getTime()))
            {
                iter.remove();
                history.addPoint(node.data, new KDPayload(node.waveBullet.guessFactor));
            }
        }



        //Gun
        double maxEscapeAngle = currentNode.waveBullet.maxEscapeAngle();
        double angleStandardDeviation = Math.asin(getWidth()/2/e.getDistance());
        final double GAUSS_FACTOR = 1.0 / Math.sqrt(2 * Math.PI);
        if (history.size() > MIN_HISTORY_TO_FIRE)
        {
            ArrayList<KDTree.SearchResult<KDPayload>> results = history.nearestNeighbours(positionInfo, Math.min(history.size(), NEIGHBOR_COUNT));
            Collections.sort(results, (o1, o2) -> //use 1 / (1+x) to weight
            {
                double res = o2.distance - o1.distance;
                if (res == 0)
                {
                    return 0;
                }
                return res > 0 ? 1 : -1;
            });
            //TODO
            double bestAngle=0;
            double bestDensity = 0;
            for (int i = 0; i < results.size(); i++)
            {
                KDPayload payloadA = results.get(i).payload;
                double density = 0;
                double angleA = getAngleFromGuessFactor(direction, payloadA.guessFactor, maxEscapeAngle);
                for (KDTree.SearchResult<KDPayload> resultB : results)
                {
                    KDPayload payloadB = resultB.payload;
                    if (payloadA != payloadB)
                    {
                        double angleB = getAngleFromGuessFactor(direction, payloadB.guessFactor, maxEscapeAngle);
                        double dAngle = (angleB - angleA)/angleStandardDeviation;
                        double strength = GAUSS_FACTOR * Math.exp(-0.5 * (dAngle * dAngle));
                        density += strength;
                    }
                }
                if (density > bestDensity)
                {
                    bestAngle = angleA;
                    bestDensity = density;
                }
            }
            double gunAdjust = Utils.normalRelativeAngle(absoluteBearing - getGunHeadingRadians()+ bestAngle);
            setTurnGunRightRadians(gunAdjust);
            setFireBullet(bulletPower);
        }
        /*
        gun.onScannedRobot(e, getHeadingRadians(), getX(), getY(), getTime(), bulletPower, getGunHeadingRadians());
        setTurnGunRightRadians(gun.gunAdjust);
        gun.postFire(setFireBullet(bulletPower) != null);*/
        /*
        double enemyX = getX() + e.getDistance() * Math.sin(absoluteBearing);
        double enemyY = getY() + e.getDistance() * Math.cos(absoluteBearing);

        double deltaTime = 0;
        double battleFieldHeight = getBattleFieldHeight();
        double battleFieldWidth = getBattleFieldWidth();
        double predictedX = enemyX;
        double predictedY = enemyY;
        while ((++deltaTime) * (20.0 - 3.0 * bulletPower) < Point2D.Double.distance(myX, myY, predictedX, predictedY))
        {
            predictedX += Math.sin(enemyHeading) * enemyVelocity;
            predictedY += Math.cos(enemyHeading) * enemyVelocity;
            enemyHeading += enemyHeadingChange;
            if (predictedX < 18.0 || predictedY < 18.0 || predictedX > battleFieldWidth - 18.0 || predictedY > battleFieldHeight - 18.0)
            {
                predictedX = Math.min(Math.max(18.0, predictedX),  battleFieldWidth - 18.0);
                predictedY = Math.min(Math.max(18.0, predictedY), battleFieldHeight - 18.0);
                break;
            }
        }*/

        //Surfing
        double enemyBulletPower = _oppEnergy - e.getEnergy();
        if (enemyBulletPower < 3.01 && bulletPower > 0.09 && _surfDirections.size() > 2)
        {
            EnemyWave ew = new EnemyWave();
            ew.fireTime = getTime() - 1;
            ew.bulletVelocity = bulletVelocity(enemyBulletPower);
            ew.distanceTraveled = bulletVelocity(enemyBulletPower);
            ew.direction = ((Integer) _surfDirections.get(2)).intValue();
            ew.directAngle = ((Double) _surfAbsBearings.get(2)).doubleValue();
            ew.fireLocation = (Point2D.Double) _enemyLocation.clone(); // last tick

            _enemyWaves.add(ew);
        }

        _oppEnergy = e.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _enemyLocation = project(_myLocation, absoluteBearing, e.getDistance());

        updateWaves();
        doSurfing();

        //Gun

        //double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - getX(), predictedY - getY()));
        /*
        *       Radar
         */
        double radarTurn = Utils.normalRelativeAngle(absoluteBearing - getRadarHeadingRadians() );
        double extraTurn = Math.min( Math.atan( 36.0 / e.getDistance() ), Rules.RADAR_TURN_RATE_RADIANS );
        if (radarTurn < 0)
        {
            radarTurn -= extraTurn;
        }
        else
        {
            radarTurn += extraTurn;
        }
        setTurnRadarRightRadians(Utils.normalRelativeAngle(radarTurn));
        /*setTurnGunRightRadians(Utils.normalRelativeAngle(theta - getGunHeadingRadians()));
        setFire(bulletPower);*/

        execute();
    }
    public double getAngleFromGuessFactor(int direction, double gf, double escapeAngle)
    {
        return direction * gf * escapeAngle;
    }
    public void updateWaves()
    {
        for (int x = 0; x < _enemyWaves.size(); x++)
        {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

            ew.distanceTraveled = (getTime() - ew.fireTime) * ew.bulletVelocity;
            if (ew.distanceTraveled >
                    _myLocation.distance(ew.fireLocation) + 50)
            {
                _enemyWaves.remove(x);
                x--;
            }
        }
    }

    public EnemyWave getClosestSurfableWave()
    {
        double closestDistance = 50000; // I juse use some very big number here
        EnemyWave surfWave = null;

        for (int x = 0; x < _enemyWaves.size(); x++)
        {
            EnemyWave ew = (EnemyWave) _enemyWaves.get(x);
            double distance = _myLocation.distance(ew.fireLocation)
                    - ew.distanceTraveled;

            if (distance > ew.bulletVelocity && distance < closestDistance)
            {
                surfWave = ew;
                closestDistance = distance;
            }
        }

        return surfWave;
    }

    public static int getFactorIndex(EnemyWave ew, Point2D.Double targetLocation)
    {
        double offsetAngle = (absoluteBearing(ew.fireLocation, targetLocation)
                - ew.directAngle);
        double factor = Utils.normalRelativeAngle(offsetAngle)
                / maxEscapeAngle(ew.bulletVelocity) * ew.direction;

        return (int) limit(0,
                (factor * ((BINS - 1) / 2)) + ((BINS - 1) / 2),
                BINS - 1);
    }

    public void logHit(EnemyWave ew, Point2D.Double targetLocation)
    {
        int index = getFactorIndex(ew, targetLocation);

        for (int x = 0; x < BINS; x++)
        {
            // for the spot bin that we were hit on, add 1;
            // for the bins next to it, add 1 / 2;
            // the next one, add 1 / 5; and so on...
            _surfStats[x] += 1.0 / (Math.pow(index - x, 2) + 1);
        }
    }

    @Override
    public void onHitByBullet(HitByBulletEvent e)
    {
        // If the _enemyWaves collection is empty, we must have missed the
        // detection of this wave somehow.
        if (!_enemyWaves.isEmpty())
        {
            Point2D.Double hitBulletLocation = new Point2D.Double(
                    e.getBullet().getX(), e.getBullet().getY());
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++)
            {
                EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

                if (Math.abs(ew.distanceTraveled -
                        _myLocation.distance(ew.fireLocation)) < 50
                        && Math.abs(bulletVelocity(e.getBullet().getPower())
                        - ew.bulletVelocity) < 0.001)
                {
                    hitWave = ew;
                    break;
                }
            }

            if (hitWave != null)
            {
                logHit(hitWave, hitBulletLocation);

                // We can remove this wave now, of course.
                _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
        }
    }

    public Point2D.Double predictPosition(EnemyWave surfWave, int direction)
    {
        Point2D.Double predictedPosition = (Point2D.Double) _myLocation.clone();
        double predictedVelocity = getVelocity();
        double predictedHeading = getHeadingRadians();
        double maxTurning, moveAngle, moveDir;

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do
        {    // the rest of these code comments are rozu's
            moveAngle =
                    wallSmoothing(predictedPosition, absoluteBearing(surfWave.fireLocation,
                            predictedPosition) + (direction * (Math.PI / 2)), direction)
                            - predictedHeading;
            moveDir = 1;

            if (Math.cos(moveAngle) < 0)
            {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            moveAngle = Utils.normalRelativeAngle(moveAngle);

            // maxTurning is built in like this, you can't turn more then this in one tick
            maxTurning = Math.PI / 720d * (40d - 3d * Math.abs(predictedVelocity));
            predictedHeading = Utils.normalRelativeAngle(predictedHeading
                    + limit(-maxTurning, moveAngle, maxTurning));

            // this one is nice ;). if predictedVelocity and moveDir have
            // different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
            predictedVelocity +=
                    (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
            predictedVelocity = limit(-8, predictedVelocity, 8);

            // calculate the new predicted position
            predictedPosition = project(predictedPosition, predictedHeading,
                    predictedVelocity);

            counter++;

            if (predictedPosition.distance(surfWave.fireLocation) <
                    surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                            + surfWave.bulletVelocity)
            {
                intercepted = true;
            }
        } while (!intercepted && counter < 500);

        return predictedPosition;
    }

    public double checkDanger(EnemyWave surfWave, int direction)
    {
        int index = getFactorIndex(surfWave,
                predictPosition(surfWave, direction));

        return _surfStats[index];
    }

    public void doSurfing()
    {
        EnemyWave surfWave = getClosestSurfableWave();

        if (surfWave == null)
        {
            return;
        }

        double dangerLeft = checkDanger(surfWave, -1);
        double dangerRight = checkDanger(surfWave, 1);

        double goAngle = absoluteBearing(surfWave.fireLocation, _myLocation);
        if (dangerLeft < dangerRight)
        {
            goAngle = wallSmoothing(_myLocation, goAngle - (Math.PI / 2), -1);
        } else
        {
            goAngle = wallSmoothing(_myLocation, goAngle + (Math.PI / 2), 1);
        }

        setBackAsFront(this, goAngle);
    }

    class EnemyWave
    {
        Point2D.Double fireLocation;
        long fireTime;
        double bulletVelocity, directAngle, distanceTraveled;
        int direction;

        public EnemyWave()
        {
        }
    }

    public double wallSmoothing(Point2D.Double botLocation, double angle, int orientation)
    {
        while (!_fieldRect.contains(project(botLocation, angle, WALL_STICK)))
        {
            angle += orientation * 0.05;
        }
        return angle;
    }

    public static Point2D.Double project(Point2D.Double sourceLocation,
                                         double angle, double length)
    {
        return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
                sourceLocation.y + Math.cos(angle) * length);
    }

    public static double absoluteBearing(Point2D.Double source, Point2D.Double target)
    {
        return Math.atan2(target.x - source.x, target.y - source.y);
    }

    public static double limit(double min, double value, double max)
    {
        return Math.max(min, Math.min(value, max));
    }

    public static double bulletVelocity(double power)
    {
        return (20.0 - (3.0 * power));
    }

    public static double maxEscapeAngle(double velocity)
    {
        return Math.asin(8.0 / velocity);
    }

    public static void setBackAsFront(AdvancedRobot robot, double goAngle)
    {
        double angle =
                Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
        if (Math.abs(angle) > (Math.PI / 2))
        {
            if (angle < 0)
            {
                robot.setTurnRightRadians(Math.PI + angle);
            } else
            {
                robot.setTurnLeftRadians(Math.PI - angle);
            }
            robot.setBack(100);
        } else
        {
            if (angle < 0)
            {
                robot.setTurnLeftRadians(-1 * angle);
            } else
            {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(100);
        }
    }
    class DNNNode
    {
        public double[] data;
        public WaveBullet waveBullet;

        public DNNNode(double[] data, WaveBullet waveBullet)
        {
            this.data = data;
            this.waveBullet = waveBullet;
        }
    }
    class KDPayload
    {
        public double guessFactor;

        public KDPayload(double guessFactor)
        {
            this.guessFactor = guessFactor;
        }
    }
    public static double getBulletSpeed(double power)
    {
        return 20 - Math.max(Math.min(power, 3.0), 0.1)* 3;
    }
}
