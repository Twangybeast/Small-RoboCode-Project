import robocode.*;
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
    public static double _surfStats[] = new double[]{
            0.02, 0.02, 0.02, 0.02, 0.02, 0.03, 0.03, 0.03, 0.03, 0.04, 0.04, 0.05, 0.05, 0.06, 0.07, 0.09, 0.11, 0.14, 0.19, 0.28, 0.47, 0.92, 1.95, 2.60, 1.57, 1.24, 1.41, 2.18, 1.83, 2.13, 1.27, 0.92, 1.03, 1.33, 2.14, 2.47, 3.77, 2.83, 2.98, 1.74, 1.21, 1.44, 2.50, 4.56, 2.34, 1.00, 0.54
    };
    public Point2D.Double _myLocation;     // our bot's location
    public Point2D.Double _enemyLocation;  // enemy bot's location

    public ArrayList<EnemyWave> _enemyWaves;
    public ArrayList _surfDirections;
    public ArrayList _surfAbsBearings;
    public LinkedList<Point2D.Double> enemyLocations;
    public int timeSinceDirectionChange = 0;
    public int timeSinceDeceleration = 0;
    public int lastDirection = 1;

    public static final double LESS_THAN_HALF_PI = 1.25;

    public static final double DEFAULT_BULLET_POWER = 2.0;

    public static double  _oppEnergy = 100.0;
    public static Rectangle2D.Double _fieldRect = new java.awt.geom.Rectangle2D.Double(18, 18, 764, 564);
    public static double WALL_STICK = 160;

    //Info: heading change, distance, velocity, acceleration, lateral velocity, advancing velocity, dist-10-ago, time-since-dir-change, time-since-decel
    public static int DIMENSIONS = 9;
    public static KDTree<DNNNode> history = new KDTree.WeightedManhattan<>(DIMENSIONS);
    public LinkedList<DNNNode> shortHistory = new LinkedList<>();
    public static int MIN_HISTORY_TO_FIRE = 30;
    public static int MIN_HISTORY_TO_ADD = 100;
    public static final int NEIGHBOR_COUNT = 50;
    public DNNNode lastNode = null;
    final static double GAUSS_FACTOR = 1.0 / Math.sqrt(2 * Math.PI);
    public static final double KD_DISTANCE_SD = 3.0;
    public static final double MIN_DENSITY = 6;

    GuessFactorGun gun = new GuessFactorGun();

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
        ((KDTree.WeightedManhattan)history).setWeights(new double[]{1, 5, 3, 10, 10, 2, 3, 3, 3});
        setAllColors(Color.YELLOW);
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        _enemyWaves = new ArrayList();
        _surfDirections = new ArrayList();
        _surfAbsBearings = new ArrayList();
        enemyLocations = new LinkedList<>();
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
        double myLateralVelocity = getVelocity() * Math.sin(e.getBearingRadians());


        double bulletPower = Math.min(DEFAULT_BULLET_POWER, e.getEnergy()/4);
        //double bulletPower = Math.min(Math.min(1.95, 1.95), 3.0);
        if (lowEnergy() && e.getDistance() > 250)
        {
            //bulletPower = 1;
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
        _surfDirections.add(0, new Integer((myLateralVelocity >= 0) ? 1 : -1));
        _surfAbsBearings.add(0, new Double(absoluteBearing + Math.PI));

        //Recording positions
        double enemyX = getX() + e.getDistance() * Math.sin(absoluteBearing);
        double enemyY = getY() + e.getDistance() * Math.cos(absoluteBearing);
        Point2D.Double enemyLocation =new Point2D.Double(enemyX, enemyY);
        enemyLocations.add(0, enemyLocation);
        double distLast10 = enemyLocation.distance(enemyLocations.get(Math.min(10, enemyLocations.size()-1)));//Distance traveled by enemy in last 10 ticks
        double BFT = e.getDistance()/bulletVelocity(DEFAULT_BULLET_POWER);
        double[] positionInfo = new double[]{
                Utils.normalRelativeAngle(enemyHeadingChange)/(Math.PI * 2 / 36),
                limit(0,e.getDistance()/900.0, 1),
                enemyVelocity/8,
                enemyVelocityChange/2,
                lateralVelocity/8,
                limit(0, advancingVelocity/16 + 0.5 , 1),
                limit(0, distLast10/(80), 1),
                1/(1 + 2.0 * timeSinceDirectionChange/BFT),
                1/(1 + 2.0*timeSinceDeceleration/BFT)
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
        if (enemyBulletPower < 3.01 && enemyBulletPower > 0.09 && _surfDirections.size() > 2)
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
        System.out.print("Wave Surfing Stats: ");
        for (double d : _surfStats)
        {
            System.out.printf("%.2f ", d);
        }
        System.out.println();
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
        //Gun
        double maxEscapeAngle = currentNode.waveBullet.maxEscapeAngle(bulletPower);
        double angleStandardDeviation = Math.asin(getWidth() / 2 / enemyDistance);
        ArrayList<KDTree.SearchResult<DNNNode>> results = history.nearestNeighbours(positionInfo, Math.min(history.size(), NEIGHBOR_COUNT));
        Collections.sort(results, (o1, o2) ->
        {
            double res = o2.distance - o1.distance;
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
        for (int i=0;i<enemyPredicted.size() && i < 10;i++)
        {
            PredictedPosition pos = enemyPredicted.get(i);
            g.setColor(new Color(1f,0, 0, 0.5f+0.5f*(i*1.0f/enemyPredicted.size())));
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
                g.drawRect((int)Math.round(enemyPos.x) - 18, (int)Math.round(enemyPos.y) - 18, 36, 36);
                double dist =aimGhosts.bulletPos.get(i);
                g.drawOval((int)Math.round(myLocation.x-dist), (int)Math.round(myLocation.y-dist), 2*(int)dist, 2*(int)dist);
            }
            g.setColor(new Color(0,1.0f, 1));
            g.drawLine((int)myLocation.x, (int)myLocation.y, (int)Math.round(Math.sin(aimGhosts.bulletBearing)*1000 + myLocation.x), (int)Math.round(Math.cos(aimGhosts.bulletBearing)*1000 + myLocation.y));
        }
        double max = 0;
        for (double d : _surfStats)
        {
            max = Math.max(d, max);
        }
        for (EnemyWave ew : _enemyWaves)
        {
            for (int i = 0; i < BINS; i++)
            {
                double offset = getAngleFromIndex(ew, i);
                double strength = _surfStats[i]/max;
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
    }
    public boolean lowEnergy()
    {
        return getEnergy() < 50;
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
        double offsetAngle = (absoluteBearing(ew.fireLocation, targetLocation) - ew.directAngle);
        double factor = Utils.normalRelativeAngle(offsetAngle) / maxEscapeAngle(ew.bulletVelocity) * ew.direction;

        return (int) limit(0, (factor * ((BINS - 1) / 2)) + ((BINS - 1) / 2), BINS - 1);
    }

    public static double getAngleFromIndex(EnemyWave ew, int index)
    {
        double factor = (index - (BINS-1)/2.0)/((BINS-1)/2);
        double angle =  factor * ew.direction * maxEscapeAngle(ew.bulletVelocity);
        return angle;
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
    public void onBulletHit(BulletHitEvent e)
    {
        _oppEnergy -= Rules.getBulletDamage(e.getBullet().getPower());
    }

    @Override
    public void onHitByBullet(HitByBulletEvent e)
    {
        _oppEnergy += e.getBullet().getPower() * 3;
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
    @Override
    public void onBulletHitBullet(BulletHitBulletEvent e)
    {
        if (!_enemyWaves.isEmpty())
        {
            Point2D.Double hitBulletLocation = new Point2D.Double(
                    e.getBullet().getX(), e.getBullet().getY());
            EnemyWave hitWave = null;

            // look through the EnemyWaves, and find one that could've hit us.
            for (int x = 0; x < _enemyWaves.size(); x++)
            {
                EnemyWave ew = (EnemyWave) _enemyWaves.get(x);

                if (Math.abs(ew.distanceTraveled - hitBulletLocation.distance(ew.fireLocation)) < 50 && Math.abs(bulletVelocity(e.getBullet().getPower())
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
    public Point2D.Double predictPosition(EnemyWave surfWave, int direction, boolean stop)
    {
        Point2D.Double predictedPosition = (Point2D.Double) _myLocation.clone();
        double predictedVelocity = getVelocity();
        double predictedHeading = getHeadingRadians();
        double maxTurning, moveAngle, moveDir;

        int counter = 0; // number of ticks in the future
        boolean intercepted = false;

        do
        {    // the rest of these code comments are rozu's
            moveAngle = wallSmoothing(predictedPosition, absoluteBearing(surfWave.fireLocation, predictedPosition) + (direction * (LESS_THAN_HALF_PI)), direction) - predictedHeading;
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
            if (stop)
            {
                if (Math.abs(predictedVelocity) <= 2)
                {
                    predictedVelocity = 0;
                }
                else
                {
                    predictedVelocity += (predictedVelocity > 0 ? -2 : 2);
                }
            }
            else
            {
                predictedVelocity += (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
            }
            predictedVelocity = limit(-8, predictedVelocity, 8);

            // calculate the new predicted position
            predictedPosition = project(predictedPosition, predictedHeading, predictedVelocity);

            counter++;

            if (predictedPosition.distance(surfWave.fireLocation) < surfWave.distanceTraveled + (counter * surfWave.bulletVelocity) + surfWave.bulletVelocity)
            {
                predictedPositionAtIntercept.add(new Point2D.Double(predictedPosition.x, predictedPosition.y));
                intercepted = true;
            }
        } while (!intercepted && counter < 500);

        return predictedPosition;
    }

    public double checkDanger(EnemyWave surfWave, int direction)
    {
        int index;
        if (direction == 0)
        {
            index = getFactorIndex(surfWave, predictPosition(surfWave, 1, true));
        }
        else
        {
            index = getFactorIndex(surfWave, predictPosition(surfWave, direction, false));
        }
        return _surfStats[index];
    }

    public void doSurfing()
    {
        EnemyWave surfWave = getClosestSurfableWave();

        if (surfWave == null)
        {
            return;
        }
        predictedPositionAtIntercept = new LinkedList<>();
        double dangerLeft = checkDanger(surfWave, -1);
        double dangerRight = checkDanger(surfWave, 1);
        double dangerNone = checkDanger(surfWave, 0);

        double goAngle = absoluteBearing(surfWave.fireLocation, _myLocation);

        boolean stop = dangerNone < dangerLeft && dangerNone < dangerRight;
        if (stop)
        {
            goAngle = wallSmoothing(_myLocation, goAngle + (LESS_THAN_HALF_PI), 1);
        }
        else
        {
            if (dangerLeft < dangerRight)
            {
                goAngle = wallSmoothing(_myLocation, goAngle - (LESS_THAN_HALF_PI), -1);
            } else
            {
                goAngle = wallSmoothing(_myLocation, goAngle + (LESS_THAN_HALF_PI), 1);
            }
        }

        setBackAsFront(this, goAngle, stop);
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