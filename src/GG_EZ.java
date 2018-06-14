import robocode.*;
import robocode.Robot;
import robocode.util.Utils;

import java.awt.*;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.*;

public class GG_EZ extends AdvancedRobot
{
    double oldEnemyHeading = -1;
    double oldEnemyVelocity = 0;
    public static int BINS = 47;
    public static double _surfStats[] = new double[BINS];
    public static double[] DEFAULT_SURF_STATS = new double[]{
            0.02, 0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03, 0.03, 0.04, 0.04, 0.05, 0.05, 0.06, 0.07, 0.09, 0.11, 0.14, 0.19, 0.28, 0.47, 0.92, 1.95, 2.60, 1.57, 1.24, 1.41, 2.18, 1.83, 2.13, 1.27, 0.92, 1.03, 1.33, 2.14, 2.47, 3.77, 2.83, 2.98, 1.74, 1.21, 1.44, 2.50, 4.56, 2.34, 1.00, 0.54
    };
    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _enemyLocation;  // enemy bot's location

    public ArrayList<EnemyWave> _enemyWaves;
    //My surfing
    public static final int SURFING_DIMENSIONS = 3;
            //Lateral velocity, advancing velocity, time since decel
    public static KDTree<WaveNode> surfingHistory = new KDTree.WeightedManhattan<>(SURFING_DIMENSIONS);
    public LinkedList<RobotStatus> myStatuses;
    public int myTimeSinceDecel = 0;
    public static final int SURFING_NEIGHBORS = 100;
    public static final double SURFING_DISTANCE_SD = 0.8;
    public static final int MIN_SURFING_HISTORY = 10;

    //GOTO
    public Point2D.Double _lastGoToPoint = null;
    public int surfingDirection = 1;
    public Point2D.Double goToDisplayPoint = null;



    double myLastHeading=0;
    public LinkedList<Point2D.Double> enemyLocations;
    public int timeSinceDirectionChange = 0;
    public int timeSinceDeceleration = 0;
    public int lastDirection = 1;

    public static final double LESS_THAN_HALF_PI = 1.25;

    public static final double DEFAULT_BULLET_POWER = 2.0;

    public static double  _oppEnergy = 100.0;
    public static Rectangle2D.Double _fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public static double WALL_STICK = 160;

    //Info: heading change, distance, velocity, acceleration, lateral velocity, advancing velocity, dist-10-ago, time-since-dir-change, time-since-decel, data decay
    public static int DIMENSIONS = 10;
    public static KDTree<DNNNode> history = new KDTree.WeightedManhattan<>(DIMENSIONS);
    public static long timesScanned = 0;
    public static double DECAY_FACTOR = 3500;
    public LinkedList<DNNNode> shortHistory = new LinkedList<>();
    public static int MIN_HISTORY_TO_FIRE = 30;
    public static int MIN_HISTORY_TO_ADD = 100;
    public static final int NEIGHBOR_COUNT = 50;
    public DNNNode lastNode = null;
    final static double GAUSS_FACTOR = 1.0 / Math.sqrt(2 * Math.PI);
    public static final double KD_DISTANCE_SD = 3.0;
    public static final double MIN_DENSITY = 6;

    public static boolean BULLET_HIT = false;
    //Bullet missing test
    public static final int MAX_HIT_HISTORY_SIZE = 5;
    public static LinkedList<ShootingHistory> hitHistory = new LinkedList<>();

    //Anti-miirror
    public static double mirrorValue = 0;

    //DEBUGGING
    public LinkedList<PredictedPosition> enemyPredicted = new LinkedList<>();
    public Point2D.Double aimedPosition = null;
    public AimGhosts aimGhosts = null;
    public AimGhosts lastGhost = null;
    public LinkedList<Point2D.Double> predictedPositionAtIntercept;

    //TODO
    //Insert to KDTREE after wave breaks
    //Have symmetry w/ +/- velocity
    //Fix nearest bearing
    @Override
    public void run()
    {
        ((KDTree.WeightedManhattan)history).setWeights(new double[]{1, 5, 3, 10, 10, 2, 3, 3, 3, 4});
        ((KDTree.WeightedManhattan)surfingHistory).setWeights(new double[]{10, 2, 3});
        setAllColors(Color.YELLOW);
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        _enemyWaves = new ArrayList();
        enemyLocations = new LinkedList<>();
        myStatuses = new LinkedList<>();
        myLastHeading = getHeadingRadians();
        predictedPositionAtIntercept = new LinkedList<>();
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


        double bulletPower = DEFAULT_BULLET_POWER* limit(0, getTotalHistory().getHitRate()/(0.1), 1);
        if (Double.isNaN(bulletPower))
        {
            bulletPower = Rules.MIN_BULLET_POWER;
        }
        bulletPower = Math.min(bulletPower, e.getEnergy()/4);
        bulletPower = Math.max(0.1, bulletPower);
        //double bulletPower = Math.min(Math.min(1.95, 1.95), 3.0);
        if (lowEnergy() && e.getDistance() > 250)
        {
            //bulletPower = 1;
        }
        if (!BULLET_HIT)
        {
            bulletPower = Rules.MIN_BULLET_POWER;
        }
        //bulletPower = 2;


        double myX = getX();
        double myY = getY();
        double enemyVelocity = e.getVelocity();
        double absoluteBearing = getHeadingRadians() + e.getBearingRadians();
        double enemyHeading = e.getHeadingRadians();
        double lateralVelocity = enemyVelocity * Math.sin(enemyHeading - absoluteBearing);
        double advancingVelocity = -Math.cos(enemyHeading - (absoluteBearing)) * enemyVelocity;
        if (oldEnemyHeading == -1)
        {
            oldEnemyHeading = enemyHeading;
        }
        double enemyHeadingChange = enemyHeading - oldEnemyHeading;
        oldEnemyHeading = enemyHeading;
        double enemyVelocityChange = enemyVelocity - oldEnemyVelocity;
        if (Math.abs(enemyVelocity) < Math.abs(oldEnemyVelocity))
        {
            timeSinceDeceleration = 0;
        }
        else
        {
            timeSinceDeceleration++;
        }
        oldEnemyVelocity = enemyVelocity;
        int direction = 1;
        if (e.getVelocity() != 0)
        {
            if (Math.sin(e.getHeadingRadians() - absoluteBearing) * e.getVelocity() < 0)
                direction = -1;
            else
                direction = 1;
        }
        if (direction == lastDirection)
        {
            timeSinceDirectionChange++;
        }
        else
        {
            timeSinceDirectionChange = 0;
        }
        lastDirection = direction;


        //Surfing
        double myVelocity = getVelocity();
        double myLateralVelocity = myVelocity * Math.sin(e.getBearingRadians());
        double myAdvancingVelocity = -myVelocity * Math.cos(e.getBearingRadians());
        myTimeSinceDecel++;
        if (!myStatuses.isEmpty() && myVelocity < myStatuses.get(0).velocity)
        {
            myTimeSinceDecel = 0;
        }

        myStatuses.add(0, new RobotStatus(_myLocation, Utils.normalRelativeAngle(absoluteBearing+Math.PI), myTimeSinceDecel, myVelocity, myLateralVelocity, myAdvancingVelocity, myLateralVelocity >= 0 ? 1 : -1, e.getDistance()));


        //Recording positions
        double enemyX = getX() + e.getDistance() * Math.sin(absoluteBearing);
        double enemyY = getY() + e.getDistance() * Math.cos(absoluteBearing);
        Point2D.Double enemyLocation =new Point2D.Double(enemyX, enemyY);
        enemyLocations.add(0, enemyLocation);
        double distLast10 = enemyLocation.distance(enemyLocations.get(Math.min(10, enemyLocations.size()-1)));//Distance traveled by enemy in last 10 ticks
        double BFT = e.getDistance()/bulletVelocity(DEFAULT_BULLET_POWER);
        double dataDecay = 1.0/(1+timesScanned/DECAY_FACTOR);
        timesScanned++;
        double[] positionInfo = new double[]{
                Utils.normalRelativeAngle(enemyHeadingChange)/(Math.PI * 2 / 36),
                limit(0,e.getDistance()/900.0, 1),
                enemyVelocity/8,
                enemyVelocityChange/2,
                lateralVelocity/8,
                limit(0, advancingVelocity/16 + 0.5 , 1),
                limit(0, distLast10/(80), 1),
                1/(1 + 2.0 * timeSinceDirectionChange/BFT),
                1/(1 + 2.0*timeSinceDeceleration/BFT),
                dataDecay
        };
        DNNNode currentNode = new DNNNode(positionInfo, new WaveBullet(getX(), getY(), absoluteBearing, 0, direction, getTime()), new Point2D.Double(enemyX, enemyY), enemyHeading,null);
        shortHistory.offer(currentNode);
        if (lastNode != null)
        {
            lastNode.next = currentNode;
        }
        lastNode = currentNode;

        if (shortHistory.size() > MIN_HISTORY_TO_ADD)
        {
            DNNNode node = shortHistory.poll();
            history.addPoint(node.data, node);
        }


        if (history.size() > MIN_HISTORY_TO_FIRE)
        {
            double density = aimGun(currentNode, positionInfo, bulletPower, absoluteBearing, direction, e.getDistance(), enemyHeading, new Point2D.Double(enemyX, enemyY));
            //System.out.println(density);
            if (density < MIN_DENSITY)
            {
                bulletPower = 0.5;
                //aimGun(currentNode, positionInfo, bulletPower, absoluteBearing, direction, e.getDistance(), enemyHeading, new Point2D.Double(enemyX, enemyY));
            }
        }

        //Surfing
        double enemyBulletPower = _oppEnergy - e.getEnergy();
        if (enemyBulletPower < 3.01 && enemyBulletPower > 0.09 && myStatuses.size() > 2)
        {
            EnemyWave ew = new EnemyWave();
            ew.fireTime = getTime() - 1;
            ew.bulletVelocity = bulletVelocity(enemyBulletPower);
            ew.distanceTraveled = bulletVelocity(enemyBulletPower);
            ew.direction = myStatuses.get(2).direction;
            ew.directAngle = myStatuses.get(2).directAngle;
            ew.fireLocation = (Point2D.Double) _enemyLocation.clone(); // last tick
            ew.status = myStatuses.get(2);
            ew.power = enemyBulletPower;
            _enemyWaves.add(ew);
        }

        _oppEnergy = e.getEnergy();

        // update after EnemyWave detection, because that needs the previous
        // enemy location as the source of the wave
        _enemyLocation = project(_myLocation, absoluteBearing, e.getDistance());

        updateWaves();
        doSurfing();

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

        execute();
    }
    public double aimGun(DNNNode currentNode, double[] positionInfo, double bulletPower, double absoluteBearing, int direction, double enemyDistance, double enemyHeading, Point2D.Double enemyPosition)
    {
        double myX = getX();
        double myY = getY();
        //Anti mirror
        Point2D.Double mirroredPosition;
        //Gun
        double maxEscapeAngle = currentNode.waveBullet.maxEscapeAngle(bulletPower);
        double angleStandardDeviation = Math.asin(getWidth() / 2 / enemyDistance);
        ArrayList<KDTree.SearchResult<DNNNode>> results = history.nearestNeighbours(positionInfo, Math.min(history.size(), NEIGHBOR_COUNT));
        Collections.sort(results, (o1, o2) ->
        {
            double res = o1.distance - o2.distance;
            if (res == 0)
            {
                return 0;
            }
            return res > 0 ? 1 : -1;
        });
        enemyPredicted = new LinkedList<>();//TODO Remove: Debug
        ArrayList<AimGhosts> bulletPaths = new ArrayList<>();
        ArrayList<Double> guessFactors = new ArrayList<>();
        Outer:
        for (int i = 0; i < results.size(); i++)
        {
            DNNNode node = results.get(i).payload;
            DNNNode startNode = node;
            WaveBullet waveBullet = new WaveBullet(myX, myY, absoluteBearing, bulletPower, direction, node.waveBullet.getFireTime());
            AimGhosts ghost = new AimGhosts();
            double predictedX = 0;
            double predictedY = 0;
            do
            {
                node = node.next;
                if (node == null)
                {
                    results.remove(i);
                    i--;
                    continue Outer;
                }

                double distanceTraveled = startNode.enemyPos.distance(node.enemyPos);
                double dx = node.enemyPos.x - startNode.enemyPos.x;
                double dy = node.enemyPos.y-startNode.enemyPos.y;
                double travelAngle = Utils.normalRelativeAngle(Math.atan2(dx, dy) - startNode.enemyHeading);
                //Final virtual travel angle for past history relative to robot heading

                //Now projecting to current position
                travelAngle += enemyHeading;
                predictedX = enemyPosition.x + Math.sin(travelAngle) * distanceTraveled;
                predictedY = enemyPosition.y + Math.cos(travelAngle) * distanceTraveled;

                //TODO: Debug
                ghost.bulletPos.add(waveBullet.getDistance(node.waveBullet.getFireTime()));
                ghost.enemyPos.add(new Point2D.Double(predictedX, predictedY));

            } while (!waveBullet.checkHit(predictedX, predictedY, node.waveBullet.getFireTime()));
            guessFactors.add(waveBullet.guessFactor);

            //TODO Remove: Debug
            bulletPaths.add(ghost);

            double distanceTraveled = startNode.enemyPos.distance(node.enemyPos);
            double dx = node.enemyPos.x - startNode.enemyPos.x;
            double dy = node.enemyPos.y-startNode.enemyPos.y;
            double travelAngle = Utils.normalRelativeAngle(Math.atan2(dx, dy) - startNode.enemyHeading);
            //Final virtual travel angle for past history relative to robot heading

            //Now projecting to current position
            travelAngle += enemyHeading;
            predictedX = enemyPosition.x + Math.sin(travelAngle) * distanceTraveled;
            predictedY = enemyPosition.y + Math.cos(travelAngle) * distanceTraveled;
            enemyPredicted.add(new PredictedPosition(predictedX, predictedY, getGaussian(results.get(i).distance, KD_DISTANCE_SD)));
        }

            /*if (!results.isEmpty())
                System.out.printf("Best distance: %f\n", results.get(0).distance);
            //TODO*/
        double bestAngle = 0;
        double bestDensity = 0;
        //TODO Rmove
        int bestIndex = 0;
        AimGhosts ghost = null;
        for (int i = 0; i < results.size(); i++)
        {
            DNNNode payloadA = results.get(i).payload;
            double density = 0;
            double angleA = getAngleFromGuessFactor(direction, guessFactors.get(i), maxEscapeAngle);
            for (int j = 0; j < results.size(); j++)
            {
                DNNNode payloadB = results.get(j).payload;
                if (payloadA != payloadB)
                {
                    double angleB = getAngleFromGuessFactor(direction, guessFactors.get(j), maxEscapeAngle);
                    double dAngle = (angleB - angleA) / angleStandardDeviation;
                    double strength = GAUSS_FACTOR * Math.exp(-0.5 * (dAngle * dAngle));
                    density += strength * getGaussian(results.get(j).distance, KD_DISTANCE_SD);
                }
            }
            if (density > bestDensity)
            {
                //TODO Remove: Debug
                double angle = Utils.normalAbsoluteAngle(absoluteBearing + angleA);
                double dist = Point2D.Double.distance(payloadA.waveBullet.getStartX(), payloadA.waveBullet.getStartY(), payloadA.enemyPos.x, payloadA.enemyPos.y);
                double px = myX + dist * Math.sin(angle);
                double py = myY + dist * Math.cos(angle);
                aimedPosition = new Point2D.Double(px, py);

                bestAngle = angleA;
                bestDensity = density;
                bestIndex = i;

                ghost = bulletPaths.get(i);
                ghost.myLocation = new Point2D.Double(_myLocation.x, _myLocation.y);
                ghost.bulletBearing = angle;
            }
        }
        double gunAdjust = Utils.normalRelativeAngle(absoluteBearing - getGunHeadingRadians() + bestAngle);
        setTurnGunRightRadians(gunAdjust);
        if(setFireBullet(bulletPower)!=null)
        {
            aimGhosts = lastGhost;
        }
        lastGhost = ghost;
        return bestDensity;
    }

    public static double getGaussian(double val, double sd)
    {
        val /= sd;
        val = val * val;
        return GAUSS_FACTOR * Math.exp(- 0.5 * val);
    }
    @Override
    public void onPaint(Graphics2D g)
    {
        g.setColor(Color.RED);
        for (int i=0;i<enemyPredicted.size() && i < 50;i++)
        {
            PredictedPosition pos = enemyPredicted.get(i);
            g.setColor(new Color(1f,0, 0, (float) pos.strength));
            g.drawRect((int)Math.round(pos.x) - 18, (int)Math.round(pos.y) - 18, 36, 36);
        }
        if (aimedPosition != null)
        {
            g.setColor(Color.GREEN);
            int radius = 3;
            g.fillOval((int)Math.round(aimedPosition.x)-radius, (int)Math.round(aimedPosition.y)-radius, radius*2, radius*2);
        }
        if (aimGhosts != null)
        {
            Point2D.Double myLocation = aimGhosts.myLocation;
            for (int i = 0; i < aimGhosts.enemyPos.size(); i+= 2)
            {
                g.setColor(new Color(0 , 1, 0f, 0.5f+0.5f*(i*1.0f/aimGhosts.enemyPos.size())));
                Point2D.Double enemyPos = aimGhosts.enemyPos.get(i);
                //g.drawRect((int)Math.round(enemyPos.x) - 18, (int)Math.round(enemyPos.y) - 18, 36, 36);
                double dist =aimGhosts.bulletPos.get(i);
                //g.drawOval((int)Math.round(myLocation.x-dist), (int)Math.round(myLocation.y-dist), 2*(int)dist, 2*(int)dist);
            }
            g.setColor(new Color(0,1.0f, 1));
            g.drawLine((int)myLocation.x, (int)myLocation.y, (int)Math.round(Math.sin(aimGhosts.bulletBearing)*1000 + myLocation.x), (int)Math.round(Math.cos(aimGhosts.bulletBearing)*1000 + myLocation.y));
        }
        for (EnemyWave ew : _enemyWaves)
        {
            if (ew.surfStats == null)
            {
                ew.surfStats = regenerateBins(ew.status, ew.fireLocation.distance(_myLocation), ew.power);
            }
            double max = 0;
            for (double d : ew.surfStats)
            {
                max = Math.max(d, max);
            }
            for (int i = 0; i < BINS; i++)
            {
                double offset = getAngleFromIndex(ew, i);
                double strength = ew.surfStats[i]/max;
                if (Double.isNaN(strength))
                {
                    strength = 0;
                }
                strength = limit(0, strength, 1);
                g.setColor(new Color((float)limit(0, strength,1), 0, (float)limit(0,(1-strength), 1)));
                double angle = ew.directAngle+offset;
                Point2D.Double point = project(ew.fireLocation, angle, ew.distanceTraveled);
                final double size = 3;
                g.fillOval(((int) (point.x - size)), ((int) (point.y + size)), (int)(size*2),(int)(size*2));
            }
        }
        for (Point2D.Double pos : predictedPositionAtIntercept)
        {
            g.setColor(Color.WHITE);
            g.drawRect(((int) (pos.x - 18)), ((int) (pos.y - 18)), 36, 36);
        }
        if (goToDisplayPoint != null)
        {
            g.setColor(Color.WHITE);
            g.drawRect(((int) (goToDisplayPoint.x - 18)), ((int) (goToDisplayPoint.y - 18)), 36, 36);
        }
    }
    public boolean lowEnergy()
    {
        return getEnergy() < 50;
    }
    public double getAngleFromGuessFactor(int direction, double gf, double escapeAngle)
    {
        return direction * gf * escapeAngle;
    }
    public double[] regenerateBins(RobotStatus status, double distance, double power)
    {
        double[] stats = new double[BINS];
        ArrayList<KDTree.SearchResult<WaveNode>> neighbors = surfingHistory.nearestNeighbours(getSurfingDataPoint(status, power), Math.min(SURFING_NEIGHBORS, surfingHistory.size()));
        if (neighbors.size() <= MIN_SURFING_HISTORY)
        {
            for (int i = 0; i < BINS; i++)
            {
                stats[i] = DEFAULT_SURF_STATS[i];
            }
            return stats;
        }

        double botWidthAngle = Math.abs(36.0/distance);

        double minDistance = 100;
        for (KDTree.SearchResult<WaveNode> neighbor : neighbors)
        {
            minDistance = Math.min(minDistance, neighbor.distance);
        }
        double distanceDivisor = 1;
        if (minDistance > 35)
        {
            distanceDivisor = minDistance / 35.0;
        }
        for (KDTree.SearchResult<WaveNode> neighbor : neighbors)
        {
            WaveNode wave = neighbor.payload;
            double strength = getGaussian(neighbor.distance/distanceDivisor, SURFING_DISTANCE_SD);
            double gf = wave.gf;
            double angle = getAngleFromFactor(gf, wave.direction, getBulletSpeed(wave.power));
            for (int x = 0; x < BINS; x++)
            {
                double binAngle = getAngleFromIndex(wave.direction, getBulletSpeed(wave.power), x);
                //_surfStats[x] += 1.0 / (Math.pow(index - x, 2) + 1);
                stats[x] += strength * getGaussian(angle - binAngle, botWidthAngle);
            }
        }
        return stats;
    }
    public void updateWaves()
    {
        for (int x = 0; x < _enemyWaves.size(); x++)
        {
            EnemyWave ew = _enemyWaves.get(x);

            ew.distanceTraveled = (getTime() - ew.fireTime) * ew.bulletVelocity;
            if (ew.distanceTraveled > _myLocation.distance(ew.fireLocation) + 50)
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
        return getFactorIndex(ew.fireLocation, ew.directAngle, ew.bulletVelocity, ew.direction, targetLocation);
    }
    public static int getFactorIndex(Point2D.Double fireLocation, double directAngle, double bulletVelocity, int direction, Point2D.Double targetLocation)
    {
        double offsetAngle = (absoluteBearing(fireLocation, targetLocation) - directAngle);
        return getFactorIndex(offsetAngle, bulletVelocity, direction);
    }
    public static int getFactorIndex(double offsetAngle, double bulletVelocity, int direction)
    {
        double factor = getFactor(offsetAngle, bulletVelocity, direction);

        return getFactorIndex(factor);
    }
    public static int getFactorIndex(double factor)
    {
        return (int) limit(0, (factor * ((BINS - 1) / 2)) + ((BINS - 1) / 2), BINS - 1);
    }
    public static double getFactor(double offsetAngle, double bulletVelocity, int direction)
    {
        return Utils.normalRelativeAngle(offsetAngle) / maxEscapeAngle(bulletVelocity) * direction;
    }

    public static double getAngleFromIndex(EnemyWave ew, int index)
    {
        return getAngleFromIndex(ew.direction, ew.bulletVelocity, index);
    }
    public static double getAngleFromIndex(int direction, double bulletVelocity, int index)
    {
        double factor = (index - (BINS-1)/2.0)/((BINS-1)/2);
        return getAngleFromFactor(factor, direction, bulletVelocity);
    }
    public static double getAngleFromFactor(double factor, int direction, double bulletVelocity)
    {
        double angle =  factor * direction * maxEscapeAngle(bulletVelocity);
        return angle;
    }
    public double[] getSurfingDataPoint(RobotStatus status, double bulletPower)
    {
        return new double[]{status.lateralVelocity/8, limit(0, status.advancingVelocity/16 + 0.5 , 1), 1/(1 + 2.0*status.timeSinceDecel/(status.distance/getBulletSpeed(bulletPower))),
        };
    }
    public void logHit(EnemyWave ew, Point2D.Double targetLocation)
    {
        int index = getFactorIndex(ew, targetLocation);
        double offsetAngle = (absoluteBearing(ew.fireLocation, targetLocation) - ew.directAngle);
        double guessFactor = getFactor(offsetAngle, ew.bulletVelocity, ew.direction);
        for (int x = 0; x < BINS; x++) {
            // for the spot bin that we were hit on, add 1;
            // for the bins next to it, add 1 / 2;
            // the next one, add 1 / 5; and so on...
            DEFAULT_SURF_STATS[x] += 1.0 / (Math.pow(index - x, 2) + 1);
        }
        RobotStatus status = ew.status;
        surfingHistory.addPoint(getSurfingDataPoint(status, ew.power), new WaveNode(offsetAngle,ew.power, guessFactor, status.lateralVelocity, status.direction));
    }
    @Override
    public void onBulletHit(BulletHitEvent e)
    {
        BULLET_HIT = true;
        _oppEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
        logBulletHitMiss().addHit();
    }
    @Override
    public void onBulletMissed(BulletMissedEvent e)
    {
        logBulletHitMiss().addMiss();
    }
    public ShootingHistory logBulletHitMiss()
    {
        ShootingHistory hist;
        if (hitHistory.isEmpty())
        {
            hist = new ShootingHistory(0,0);
            hitHistory.add(hist);
            return hist;
        }
        else
        {
            hist = hitHistory.getLast();
        }
        if (hist.total == ShootingHistory.HISTORY_SIZE)
        {
            hist = new ShootingHistory(0,0);
            hitHistory.addLast(hist);
            if(hitHistory.size() > MAX_HIT_HISTORY_SIZE)
            {
                hitHistory.removeFirst();
            }
            return hist;
        }
        else
        {
            return hist;
        }
    }
    public ShootingHistory getTotalHistory()
    {
        int hit = 0;
        int total = 0;
        for (ShootingHistory hist : hitHistory)
        {
            hit += hist.numHit;
            total += hist.total;
        }
        if (total == 0)
        {
            total = 1;
        }
        return new ShootingHistory(hit, total);
    }
    @Override
    public void onHitByBullet(HitByBulletEvent e)
    {
        _oppEnergy += e.getBullet().getPower() * 3;
        onBulletDataPoint(new Point2D.Double(e.getBullet().getX(), e.getBullet().getY()), e.getBullet().getPower());
    }
    @Override
    public void onBulletHitBullet(BulletHitBulletEvent e)
    {
        onBulletDataPoint(new Point2D.Double(e.getBullet().getX(), e.getBullet().getY()), e.getBullet().getPower());
    }
    public void onBulletDataPoint(Point2D.Double hitLocation, double power)
    {
        if (!_enemyWaves.isEmpty())
        {
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++)
            {
                EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

                if (Math.abs(ew.distanceTraveled - hitLocation.distance(ew.fireLocation)) < 50 && Math.abs(bulletVelocity(power) - ew.bulletVelocity) < 0.001)
                {
                    hitWave = ew;
                    break;
                }
            }

            if (hitWave != null)
            {
                logHit(hitWave, hitLocation);

                // We can remove this wave now, of course.
                _enemyWaves.remove(_enemyWaves.lastIndexOf(hitWave));
            }
            else
            {
                System.out.println("Couldn't find bullet");
            }
        }
    }
    // CREDIT: mini sized predictor from Apollon, by rozu
    // http://robowiki.net?Apollon
    public ArrayList predictPositions(EnemyWave surfWave, int direction) {
        Point2D.Double predictedPosition = (Point2D.Double)_myLocation.clone();
        double predictedVelocity = getVelocity();
        double predictedHeading = getHeadingRadians();
        double maxTurning, moveAngle, moveDir;
        ArrayList traveledPoints = new ArrayList();

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do {
            double distance = predictedPosition.distance(surfWave.fireLocation);
            double offset = Math.PI/2 - 1 + distance/400;

            moveAngle =
                    wallSmoothing(predictedPosition, absoluteBearing(surfWave.fireLocation,
                            predictedPosition) + (direction * (offset)), direction)
                            - predictedHeading;
            moveDir = 1;

            if(Math.cos(moveAngle) < 0) {
                moveAngle += Math.PI;
                moveDir = -1;
            }

            moveAngle = Utils.normalRelativeAngle(moveAngle);

            // maxTurning is built in like this, you can't turn more then this in one tick
            maxTurning = Math.PI/720d*(40d - 3d*Math.abs(predictedVelocity));
            predictedHeading = Utils.normalRelativeAngle(predictedHeading
                    + limit(-maxTurning, moveAngle, maxTurning));

            // this one is nice ;). if predictedVelocity and moveDir have
            // different signs you want to breack down
            // otherwise you want to accelerate (look at the factor "2")
            predictedVelocity += (predictedVelocity * moveDir < 0 ? 2*moveDir : moveDir);
            predictedVelocity = limit(-8, predictedVelocity, 8);

            // calculate the new predicted position
            predictedPosition = project(predictedPosition, predictedHeading, predictedVelocity);

            //add this point the our prediction
            traveledPoints.add(predictedPosition);

            counter++;

            if (predictedPosition.distance(surfWave.fireLocation) - 20 <
                    surfWave.distanceTraveled + (counter * surfWave.bulletVelocity)
                //   + surfWave.bulletVelocity
                    ) {
                intercepted = true;
            }
        } while(!intercepted && counter < 500);

        //we can't get the the last point, because we need to slow down
        if(traveledPoints.size() > 1)
            traveledPoints.remove(traveledPoints.size() - 1);

        return traveledPoints;
    }

    public double checkDanger(EnemyWave surfWave, Point2D.Double position) {
        int index = getFactorIndex(surfWave, position);
        double distance = position.distance(surfWave.fireLocation);
        return _surfStats[index]/distance;
    }

    public Point2D.Double getBestPoint(EnemyWave surfWave){
        if(surfWave.safePoints == null){
            ArrayList forwardPoints = predictPositions(surfWave, 1);
            ArrayList reversePoints = predictPositions(surfWave, -1);
            int FminDangerIndex = 0;
            int RminDangerIndex = 0;
            double FminDanger = Double.POSITIVE_INFINITY;
            double RminDanger = Double.POSITIVE_INFINITY;
            for(int i = 0, k = forwardPoints.size(); i < k; i++){
                double thisDanger = checkDanger(surfWave, (Point2D.Double)(forwardPoints.get(i)));
                if(thisDanger <= FminDanger){
                    FminDangerIndex = i;
                    FminDanger = thisDanger;
                }
            }
            for(int i = 0, k = reversePoints.size(); i < k; i++){
                double thisDanger = checkDanger(surfWave, (Point2D.Double)(reversePoints.get(i)));
                if(thisDanger <= RminDanger){
                    RminDangerIndex = i;
                    RminDanger = thisDanger;
                }
            }
            ArrayList bestPoints;
            int minDangerIndex;

            if(FminDanger < RminDanger ){
                bestPoints = forwardPoints;
                minDangerIndex = FminDangerIndex;
            }
            else {
                bestPoints = reversePoints;
                minDangerIndex = RminDangerIndex;
            }

            Point2D.Double bestPoint = (Point2D.Double)bestPoints.get(minDangerIndex);
            goToDisplayPoint = bestPoint;
            while(bestPoints.indexOf(bestPoint) != -1)
                bestPoints.remove(bestPoints.size() - 1);
            bestPoints.add(bestPoint);

            surfWave.safePoints = bestPoints;

            //debugging - so that we should always be on top of the last point
            bestPoints.add(0,new Point2D.Double(getX(), getY()));

        }
        else
        if(surfWave.safePoints.size() > 1)
            surfWave.safePoints.remove(0);


        if(surfWave.safePoints.size() >= 1){
            for(int i = 0,k=surfWave.safePoints.size(); i < k; i++){
                Point2D.Double goToPoint = (Point2D.Double)surfWave.safePoints.get(i);
                if(goToPoint.distanceSq(_myLocation) > 20*20*1.1)
                    //if it's not 20 units away we won't reach max velocity
                    return goToPoint;
            }
            //if we don't find a point 20 units away, return the end point
            return (Point2D.Double)surfWave.safePoints.get(surfWave.safePoints.size() - 1);


        }

        return null;
    }

    public void doSurfing() {
        EnemyWave surfWave = getClosestSurfableWave();
        double distance = _enemyLocation.distance(_myLocation);
        if (surfWave == null || distance < 50) {
            //do 'away' movement  best distance of 400 - modified from RaikoNano
            double absBearing = absoluteBearing(_myLocation, _enemyLocation);
            double headingRadians = getHeadingRadians();
            double stick = 160;//Math.min(160,distance);
            double  v2, offset = Math.PI/2 + 1 - distance/400;

            while(!_fieldRect.
                    contains(project(_myLocation,v2 = absBearing + surfingDirection*(offset -= 0.02), stick)

                            // 	getX() + stick * Math.sin(v2 = absBearing + direction * (offset -= .02)), getY() + stick * Math.cos(v2)
                    ));


            if( offset < Math.PI/3 )
                surfingDirection = -surfingDirection;
            setAhead(50*Math.cos(v2 - headingRadians));
            setTurnRightRadians(Math.tan(v2 - headingRadians));

        }
        else
        {
            _surfStats = regenerateBins(surfWave.status, surfWave.fireLocation.distance(_myLocation), surfWave.power);
            goTo(getBestPoint(surfWave));
        }
    }
    private void goTo(Point2D.Double destination) {
        if(destination == null){
            if(_lastGoToPoint != null)
                destination = _lastGoToPoint;
            else
                return;
        }

        _lastGoToPoint = destination;
        Point2D.Double location = new Point2D.Double(getX(), getY());
        double distance = location.distance(destination);
        double angle = Utils.normalRelativeAngle(absoluteBearing(location, destination) - getHeadingRadians());
        if (Math.abs(angle) > Math.PI/2) {
            distance = -distance;
            if (angle > 0) {
                angle -= Math.PI;
            }
            else {
                angle += Math.PI;
            }
        }

        //this is hacked so that the bot doesn't turn once we get to our destination
        setTurnRightRadians(angle*Math.signum(Math.abs((int)distance)));

        setAhead(distance);
    }

    class EnemyWave
    {
        Point2D.Double fireLocation;
        long fireTime;
        double bulletVelocity, directAngle, distanceTraveled;
        int direction;
        public RobotStatus status;
        public double power;
        public ArrayList<Point2D.Double> safePoints;
        double[] surfStats = null;

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

    public static void setBackAsFront(AdvancedRobot robot, double goAngle, boolean stop)
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
            robot.setBack(stop ? 0 : 100);
        } else
        {
            if (angle < 0)
            {
                robot.setTurnLeftRadians(-1 * angle);
            } else
            {
                robot.setTurnRightRadians(angle);
            }
            robot.setAhead(stop ? 0 : 100);
        }
    }
    class DNNNode
    {
        public double[] data;
        public WaveBullet waveBullet;
        public Point2D.Double enemyPos;
        public double enemyHeading;
        public DNNNode next;

        public DNNNode(double[] data, WaveBullet waveBullet, Point2D.Double enemyPos, double enemyHeading, DNNNode next)
        {
            this.data = data;
            this.waveBullet = waveBullet;
            this.enemyPos = enemyPos;
            this.enemyHeading = enemyHeading;
            this.next = next;
        }
    }
    class WaveNode
    {
        public double relativeAimedAngle;
        public double power;
        public double gf;
        public double lateralvelocity;
        public int direction;

        public WaveNode(double relativeAimedAngle, double power, double gf, double lateralvelocity, int direction)
        {
            this.relativeAimedAngle = relativeAimedAngle;
            this.power = power;
            this.gf = gf;
            this.lateralvelocity = lateralvelocity;
            this.direction = direction;
        }
    }
    public static double getBulletSpeed(double power)
    {
        return 20 - Math.max(Math.min(power, 3.0), 0.1)* 3;
    }
    class AimGhosts
    {
        public ArrayList<Double> bulletPos;
        public ArrayList<Point2D.Double> enemyPos;
        public double bulletBearing = 0.0;
        public Point2D.Double myLocation = null;
        public AimGhosts()
        {
            this.bulletPos = new ArrayList<>();
            this.enemyPos = new ArrayList<>();
        }
    }
}

class PredictedPosition
{
    double x;
    double y;
    double strength;

    public PredictedPosition(double x, double y, double strength)
    {
        this.x = x;
        this.y = y;
        this.strength = strength;
    }
}
class ShootingHistory
{
    public static final int HISTORY_SIZE = 4;
    public int numHit;
    public int total;

    public ShootingHistory(int numHit, int total)
    {
        this.numHit = numHit;
        this.total = total;
    }
    public void addHit()
    {
        numHit++;
        total++;
    }
    public void addMiss()
    {
        total++;
    }
    public double getHitRate()
    {
        return numHit * 1.0 / total;
    }
}
class RobotStatus
{
    public Point2D.Double location;
    public double directAngle;
    public int timeSinceDecel;
    public double velocity;
    public double lateralVelocity;
    public double advancingVelocity;
    public int direction;
    public double distance;

    public RobotStatus(Point2D.Double location, double directAngle, int timeSinceDecel, double velocity, double lateralVelocity, double advancingVelocity, int direction, double distance)
    {
        this.location = location;
        this.directAngle = directAngle;
        this.timeSinceDecel = timeSinceDecel;
        this.velocity = velocity;
        this.lateralVelocity = lateralVelocity;
        this.advancingVelocity = advancingVelocity;
        this.direction = direction;
        this.distance = distance;
    }
}