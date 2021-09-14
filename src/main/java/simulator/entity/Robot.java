package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Robot extends JComponent {
    private static final double THETA_CAR = 270; // this is facing up
    private static final double DIRECTION_MARGIN_OF_ERROR = 1;
    public final double MAX_TURNING_RADIUS;
    public final int TARGET_FPS = 120;
    private final Rectangle2D body;
    private final Rectangle2D leftWheel;
    private final Rectangle2D rightWheel;
    private final Dimension size;
    private final double distanceBetweenFrontBackWheels;
    private final double distancePerTick; // the distance the robot moves per tick
    private final double angleChangePerClick;
    private final double MAX_TURNING_ANGLE = 45;
    private final Logger logger;
    private Point startingPoint;
    private Point currentLocation;
    private Point currentLocationCenter;
    private AffineTransform bodyAffineTransform = new AffineTransform();
    private AffineTransform leftWheelAffineTransform = new AffineTransform();
    private AffineTransform rightWheelAffineTransform = new AffineTransform();
    // since the coordinate system is positive x is right, positive y is down,
    // angles work by starting from positive x and going clockwise
    private double speed = 10 * 3; // 3 is the environment scaling factor (lazy so I just manually set here)
    private double directionInDegrees = -90;
    // need to incorporate concept of speed later
    private double turningRadius = 0;
    private double turningAngleDegrees = 0;
    private double thetaWheelsDegree = 0; // with reference to the front of car
    private int avoidingPhase = 0;
    private Direction directionToTurnFrom = Direction.NONE;
    private Direction directionToTurn = Direction.NONE;
    private Rectangle2D avoidingObstacle;
    private boolean firstTime = true;

    private long startTime;
    private boolean started = false;
    private ArrayList<String> movementQueue = new ArrayList<>();
    private ArrayList<Long> durationQueue = new ArrayList<>(); // in seconds?
    private ArrayList<Direction> directionQueue = new ArrayList<>(); // in seconds?

    public Robot(Dimension size, Point startingPoint, double distanceBetweenFrontBackWheels) {
        logger = Logger.getLogger(Robot.class.getName());

        this.size = size;
        this.startingPoint = startingPoint;
        this.setCurrentLocationCenter(startingPoint);
        body = new Rectangle2D.Double(0, 0, size.getWidth(), size.getHeight());
        leftWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);
        rightWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);

        this.distanceBetweenFrontBackWheels = distanceBetweenFrontBackWheels;
        distancePerTick = this.speed / this.TARGET_FPS;
        angleChangePerClick = 5;

        this.MAX_TURNING_RADIUS = Math
                .abs(this.distanceBetweenFrontBackWheels * Math.tan(Math.toRadians(this.MAX_TURNING_ANGLE)));

        setOpaque(false);
    }

    public void reset() {
        this.setCurrentLocationCenter(this.startingPoint);
        firstTime = true;
        bodyAffineTransform = new AffineTransform();
        leftWheelAffineTransform = new AffineTransform();
        rightWheelAffineTransform = new AffineTransform();
        directionInDegrees = -90;
        turningRadius = 0;
        turningAngleDegrees = 0;
        thetaWheelsDegree = 0;
        avoidingPhase = 0;
        directionToTurnFrom = Direction.NONE;
        directionToTurn = Direction.NONE;
        avoidingObstacle = null;

        this.repaint();
    }

    public double getTwoTurnsDistance() {
        return MAX_TURNING_RADIUS * 2.4;
    }

    // can combine with turnRight and accept one parameter for the angle to turn
    public void turnLeft() {
        if (this.thetaWheelsDegree > -this.MAX_TURNING_ANGLE) {
            this.thetaWheelsDegree -= angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(-angleChangePerClick));
            doWheelsTurned();
            this.repaint();
        }
    }

    public void turnRight() {
        if (this.thetaWheelsDegree < this.MAX_TURNING_ANGLE) {
            this.thetaWheelsDegree += angleChangePerClick;
            this.leftWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            this.rightWheelAffineTransform.rotate(Math.toRadians(angleChangePerClick));
            doWheelsTurned();
            this.repaint();
        }
    }

    public void moveForward() {
        double angleDegrees = THETA_CAR + turningAngleDegrees;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angleDegrees));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        this.bodyAffineTransform.rotate(Math.toRadians(this.turningAngleDegrees));
        directionInDegrees += this.turningAngleDegrees;
        if (directionInDegrees > 180) {
            directionInDegrees -= 360;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 360;
        }

        logger.log(Level.FINE, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = "
                + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    public void moveBackward() {
        double angleDegrees = THETA_CAR - 180 - this.turningAngleDegrees;

        double dX = this.distancePerTick * Math.cos(Math.toRadians(angleDegrees));
        double dY = this.distancePerTick * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        this.bodyAffineTransform.rotate(-Math.toRadians(this.turningAngleDegrees));
        directionInDegrees -= this.turningAngleDegrees;
        if (directionInDegrees > 180) {
            directionInDegrees -= 360;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 360;
        }

        logger.log(Level.FINE, "X = " + this.bodyAffineTransform.getTranslateX() + ", Y = "
                + this.bodyAffineTransform.getTranslateY());

        this.repaint();
    }

    public boolean moveForwardWithChecking(MyPoint myPoint, int distanceMarginOfError, Direction finalDirection,
                                           ComplexInstruction instruction, Rectangle2D[] obstacles) {
        if (instruction.getDistance() == Double.MIN_VALUE) {
            switch (finalDirection) {
            case NORTH:
            case SOUTH:
                if (Math.abs(myPoint.getY() - getCurrentLocation().getY()) < this.MAX_TURNING_RADIUS) {
                    return true;
                }
                break;
            case EAST:
            case WEST:
                if (Math.abs(myPoint.getX() - getCurrentLocation().getX()) < this.MAX_TURNING_RADIUS) {
                    return true;
                }
                break;
            case NONE:
                if (Math.abs(myPoint.getX() - getCurrentLocation().getX()) < distanceMarginOfError / 2.0
                        && (Math.abs(myPoint.getY() - getCurrentLocation().getY()) < distanceMarginOfError / 2.0)) {
                    return true;
                }
                break;
            }
        } else {
            if (instruction.getDistance() < 0) {
                return true;
            }
            instruction.subtractDistance(distancePerTick);
        }
        moveForward();
        return false;
    }

    public boolean moveBackwardWithChecking(MyPoint myPoint, int distanceMarginOfError, Direction finalDirection,
                                            ComplexInstruction instruction) {
        if (instruction.getDistance() < 0) {
            return true;
        }
        instruction.subtractDistance(distancePerTick);

        moveBackward();
        return false;
    }

    public boolean moveForwardLeftWithChecking(Direction finalDirection) {
        // Full left turn
        for (int i = 0; i < 9; i++) {
            turnLeft();
        }
        switch (finalDirection) {
        case NORTH:
            if (Math.abs(directionInDegrees - (-90)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnRight();
                }
                return true;
            }
            break;
        case SOUTH:
            if (Math.abs(directionInDegrees - (90)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnRight();
                }
                return true;
            }
            break;
        case EAST:
            if (Math.abs(directionInDegrees) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnRight();
                }
                return true;
            }
            break;
        case WEST:
            if (Math.abs(directionInDegrees - 180) <= DIRECTION_MARGIN_OF_ERROR
                    || Math.abs(directionInDegrees - (-180)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnRight();
                }
                return true;
            }
            break;
        }
        moveForward();
        return false;
    }

    public boolean moveForwardRightWithChecking(Direction finalDirection) {
        // Full right turn
        for (int i = 0; i < 9; i++) {
            turnRight();
        }
        switch (finalDirection) {
        case NORTH:
            if (Math.abs(directionInDegrees - (-90)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnLeft();
                }
                return true;
            }
            break;
        case SOUTH:
            if (Math.abs(directionInDegrees - (90)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnLeft();
                }
                return true;
            }
            break;
        case EAST:
            if (Math.abs(directionInDegrees) % 360 <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnLeft();
                }
                return true;
            }
            break;
        case WEST:
            if (Math.abs(directionInDegrees - 180) <= DIRECTION_MARGIN_OF_ERROR
                    || Math.abs(directionInDegrees - (-180)) <= DIRECTION_MARGIN_OF_ERROR) {
                for (int i = 0; i < 9; i++) {
                    turnLeft();
                }
                return true;
            }
            break;
        }
        moveForward();
        return false;
    }

    public void letsGo(double frames) {
        if (!this.durationQueue.isEmpty()) {
            long timeDelta = (long) (1000000000 * frames / TARGET_FPS);
            long durationLeft = this.durationQueue.get(0);

            if (durationLeft > 0) {
                this.durationQueue.set(0, durationLeft - timeDelta);
                // System.out.println(this.movementQueue.get(0) + ": " +
                // this.durationQueue.get(0) + " " + this.directionQueue.get(0));
                switch (this.movementQueue.get(0)) {
                default:
                    break;
                case "Forward":
                    this.moveForwardTime(timeDelta);
                    break;
                case "Right":
                    this.turnRightTime(timeDelta);
                    break;
                case "RF":
                    if (this.turn90WithChecking(timeDelta, this.directionQueue.get(0))) {
                        this.durationQueue.set(0, (long) -1);
                    }
                    break;
                case "Left":
                    this.turnLeftTime(timeDelta);
                    break;
                case "LF":
                    if (this.turn90WithChecking(timeDelta, this.directionQueue.get(0))) {
                        this.durationQueue.set(0, (long) -1);
                    }
                    break;
                case "Center":
                    this.turnCenterTime(timeDelta);
                    break;
                case "Reverse":
                    this.moveBackwardTime(timeDelta);
                    break;
                }
            } else {
                this.durationQueue.remove(0);
                this.movementQueue.remove(0);
                this.directionQueue.remove(0);
            }
        }
    }

    public void addToQueue(String command, double durationInSecs, Direction d) {
        movementQueue.add(command);
        durationQueue.add((long) (durationInSecs * 1000000000));
        directionQueue.add(d);
    }

    public void moveForwardTime(long timeDelta) {
        double angleDegrees = THETA_CAR + turningAngleDegrees;
        double distance = this.speed * timeDelta / 1000000000;

        double dX = distance * Math.cos(Math.toRadians(angleDegrees));
        double dY = distance * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        double rotation = (this.turningAngleDegrees);
        this.bodyAffineTransform.rotate(Math.toRadians(rotation));
        directionInDegrees += rotation;
        if (directionInDegrees > 180) {
            directionInDegrees -= 2 * 180;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 2 * 180;
        }

        this.repaint();
    }

    public void turnRightTime(long timeDelta) {
        if (this.thetaWheelsDegree < this.MAX_TURNING_ANGLE) {
            this.thetaWheelsDegree += this.MAX_TURNING_ANGLE;
            this.leftWheelAffineTransform.rotate(Math.toRadians(this.MAX_TURNING_ANGLE));
            this.rightWheelAffineTransform.rotate(Math.toRadians(this.MAX_TURNING_ANGLE));
            doWheelsTurned();
            this.repaint();
        }
    }

    public boolean turn90WithChecking(long timeDelta, Direction targetDirection) {
        switch (targetDirection) {
        case NORTH:
            if (Math.abs(directionInDegrees - (-90)) <= DIRECTION_MARGIN_OF_ERROR) {
                return true;
            }
            break;
        case SOUTH:
            if (Math.abs(directionInDegrees - (90)) <= DIRECTION_MARGIN_OF_ERROR) {
                return true;
            }
            break;
        case EAST:
            if (Math.abs(directionInDegrees) % 360 <= DIRECTION_MARGIN_OF_ERROR) {
                return true;
            }
            break;
        case WEST:
            if (Math.abs(directionInDegrees - 180) <= DIRECTION_MARGIN_OF_ERROR
                    || Math.abs(directionInDegrees - (-180)) <= DIRECTION_MARGIN_OF_ERROR) {
                return true;
            }
            break;
        }
        moveForwardTime(timeDelta);
        return false;
    }

    public void turnLeftTime(long timeDelta) {
        if (this.thetaWheelsDegree > -this.MAX_TURNING_ANGLE) {
            this.thetaWheelsDegree = -this.MAX_TURNING_ANGLE;
            this.leftWheelAffineTransform.rotate(Math.toRadians(-this.MAX_TURNING_ANGLE));
            this.rightWheelAffineTransform.rotate(Math.toRadians(-this.MAX_TURNING_ANGLE));
            doWheelsTurned();
            this.repaint();
        }

    }

    public void turnCenterTime(long timeDelta) {
        if (this.thetaWheelsDegree > 0) {
            this.thetaWheelsDegree = 0;
            this.leftWheelAffineTransform.rotate(Math.toRadians(-this.MAX_TURNING_ANGLE));
            this.rightWheelAffineTransform.rotate(Math.toRadians(-this.MAX_TURNING_ANGLE));
        }
        if (this.thetaWheelsDegree < 0) {
            this.thetaWheelsDegree = 0;
            this.leftWheelAffineTransform.rotate(Math.toRadians(this.MAX_TURNING_ANGLE));
            this.rightWheelAffineTransform.rotate(Math.toRadians(this.MAX_TURNING_ANGLE));
        }

        doWheelsTurned();
        this.repaint();
    }

    public void moveBackwardTime(long timeDelta) {
        double angleDegrees = THETA_CAR - 180 - this.turningAngleDegrees;
        double distance = this.speed * timeDelta / 1000000000;

        double dX = distance * Math.cos(Math.toRadians(angleDegrees));
        double dY = distance * Math.sin(Math.toRadians(angleDegrees));

        this.bodyAffineTransform.translate(dX, dY);

        double rotation = this.turningAngleDegrees;
        this.bodyAffineTransform.rotate(-Math.toRadians(rotation));
        directionInDegrees += rotation;
        if (directionInDegrees > 180) {
            directionInDegrees -= 2 * 180;
        } else if (directionInDegrees < -180) {
            directionInDegrees += 2 * 180;
        }

        this.repaint();
    }

    // calculate turningRadius and turningAngleDegrees
    // based on distance between front and back wheel axis
    private void doWheelsTurned() {
        this.turningRadius = this.thetaWheelsDegree == 0 ? 0
                : Math.abs(this.distanceBetweenFrontBackWheels * Math.tan(Math.toRadians(this.thetaWheelsDegree)));

        this.turningAngleDegrees = this.turningRadius == 0 ? 0
                : Math.toDegrees(this.distancePerTick / this.turningRadius);
        this.turningAngleDegrees = this.thetaWheelsDegree > 0 ? turningAngleDegrees : -turningAngleDegrees;
    }

    // using currentLocation, calculate the center point
    private Point calculateCenter() {
        return new Point((int) (currentLocation.getX() + size.getWidth() / 2),
                (int) (currentLocation.getY() + size.getHeight() / 2));
    }

    // using currentLocationCenter, calculate the top left point
    private Point calculateTopLeftPoint() {
        return new Point((int) (currentLocationCenter.getX() - size.getWidth() / 2),
                (int) (currentLocationCenter.getY() - size.getHeight() / 2));
    }

    public void paintComponent(Graphics g) {
        Graphics2D g2 = (Graphics2D) g;

        // initial transformation to starting point
        if (firstTime) {
            this.bodyAffineTransform.setToIdentity();
            this.bodyAffineTransform.translate(this.size.getWidth() / 2, this.size.getHeight() / 2);
            this.bodyAffineTransform.translate(this.currentLocation.x, this.currentLocation.y);

            this.leftWheelAffineTransform.setToIdentity();
            // for size of wheel
            this.leftWheelAffineTransform.translate(this.size.getWidth() / 10, this.size.getHeight() / 6);
            // position wheel relative to center of car
            this.leftWheelAffineTransform.translate(this.size.getWidth() / 5, this.size.getHeight() / 5);

            this.rightWheelAffineTransform.setToIdentity();
            this.rightWheelAffineTransform.translate(this.size.getWidth() / 10, this.size.getHeight() / 6);
            this.rightWheelAffineTransform.translate(3 * size.getWidth() / 5, size.getHeight() / 5);

            firstTime = false;
        }

        AffineTransform toCenterAt;

        // transform body of to the starting point
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.bodyAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 2, -this.size.getHeight() / 2);
        g2.transform(toCenterAt);

        // save this transformation to apply to the other wheels
        // so that overall car transformation will apply to them too
        AffineTransform saveXform = g2.getTransform();

        // draw car body
        g2.setColor(Color.lightGray);
        g2.fill(body);

        // draw left wheel
        //
        g2.setTransform(saveXform);
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.leftWheelAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 10, -this.size.getHeight() / 6);
        g2.transform(toCenterAt);

        g2.setColor(Color.darkGray);
        g2.fill(leftWheel);

        g2.setTransform(saveXform);
        toCenterAt = new AffineTransform();
        toCenterAt.concatenate(this.rightWheelAffineTransform);
        toCenterAt.translate(-this.size.getWidth() / 10, -this.size.getHeight() / 6);
        g2.transform(toCenterAt);

        g2.fill(rightWheel);
    }

    /**
     * Get the centre point of the robot.
     *
     * @return Point object of the centre of the robot.
     */
    public MyPoint getCurrentLocation() {
        return new MyPoint((int) this.bodyAffineTransform.getTranslateX(),
                (int) this.bodyAffineTransform.getTranslateY(), getGeneralDirection());
    }

    // when setting currentLocation, calculate and set currentLocationCenter
    public void setCurrentLocation(Point loc) {
        this.currentLocation = loc;
        this.currentLocationCenter = this.calculateCenter();
    }

    // when setting currentLocationCenter, calculate and set currentLocation
    public void setCurrentLocationCenter(Point loc) {
        this.currentLocationCenter = loc;
        this.currentLocation = this.calculateTopLeftPoint();
    }

    public AffineTransform getBodyAffineTransform() {
        return bodyAffineTransform;
    }

    public double getDirectionInDegrees() {
        return directionInDegrees;
    }

    public Direction getGeneralDirection() {
        if (directionInDegrees >= -45 && directionInDegrees <= 45) {
            return Direction.EAST;
        } else if (directionInDegrees >= -135 && directionInDegrees <= -45) {
            return Direction.NORTH;
        } else if (directionInDegrees >= 45 && directionInDegrees <= 135) {
            return Direction.SOUTH;
        } else if ((directionInDegrees >= 135 && directionInDegrees <= 180)
                || (directionInDegrees >= -180 && directionInDegrees <= -135)) {
            return Direction.WEST;
        }

        return Direction.NONE;
    }

    public Point2D getLeftEdgeCenter() {
        Point2D leftEdgeCenter = new Point2D.Double(0, 0);

        AffineTransform af = (AffineTransform) this.bodyAffineTransform.clone();
        af.translate(-this.size.getWidth() / 2, 0);
        af.transform(leftEdgeCenter, leftEdgeCenter);

        return leftEdgeCenter;
    }

    public Point2D getRightEdgeCenter() {

        Point2D rightEdgeCenter = new Point2D.Double(0, 0);

        AffineTransform af = (AffineTransform) this.bodyAffineTransform.clone();
        af.translate(this.size.getWidth() / 2, 0);
        af.transform(rightEdgeCenter, rightEdgeCenter);

        return rightEdgeCenter;
    }

    public Point2D getTopEdgeCenter() {
        Point2D topEdgeCenter = new Point2D.Double(0, 0);

        AffineTransform af = (AffineTransform) this.bodyAffineTransform.clone();
        af.translate(-this.size.getHeight() / 2, 0);
        af.transform(topEdgeCenter, topEdgeCenter);

        return topEdgeCenter;
    }

    public Point2D getBottomEdgeCenter() {
        Point2D bottomEdgeCenter = new Point2D.Double(0, 0);

        AffineTransform af = (AffineTransform) this.bodyAffineTransform.clone();
        af.translate(this.size.getHeight() / 2, 0);
        af.transform(bottomEdgeCenter, bottomEdgeCenter);

        return bottomEdgeCenter;
    }

    public RelativeDirection getRelativeDirection(MyPoint p1, MyPoint p2) {
        // find p2's relative direction from p1
        // meaning, from p1's POV, where is p2?
        MyPoint pp = (MyPoint) p2.clone();

        AffineTransform af = new AffineTransform();
        af.setToIdentity();
        af.translate((int) p1.getX(), (int) p1.getY());
        switch (p1.getDirection()) {
        case NORTH:
            af.rotate(Math.toRadians(0));
            break;
        case SOUTH:
            af.rotate(Math.toRadians(180));
            pp.rotate180();
            break;
        case EAST:
            af.rotate(Math.toRadians(90));
            pp.rotateLeft90();
            break;
        case WEST:
            af.rotate(Math.toRadians(-90));
            pp.rotateRight90();
            break;
        case NONE:
            break;
        default:
            break;

        }

        try {
            af.inverseTransform(pp, pp);
        } catch (NoninvertibleTransformException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // System.out.println("pp: " + pp.x + " " + pp.y + " " + pp.getDirection());
        // System.out.println("pp: " + pp.getY() + " " + pp.y + " " + this.getTwoTurnsDistance());

        int twoTurnDistance = (int) this.getTwoTurnsDistance();
        if (-this.size.width / 2 <= pp.getX() && this.size.width / 2 >= pp.getX()) {
            if (pp.getY() <= 0) {
                return RelativeDirection.FRONT;
            } else if (pp.getY() >= 0) {
                return RelativeDirection.BACK;
            }
        } else if (Math.abs(pp.getY()) < twoTurnDistance) {
            // within the 2 turn margin of front and back
            if (0 <= pp.getX()) {
                return RelativeDirection.CENTER_RIGHT;
            } else if (0 >= pp.getX()) {
                return RelativeDirection.CENTER_LEFT;
            }

        } else if (-twoTurnDistance >= pp.getY()) {
            // point in front within two turn margin
            if (pp.getX() <= twoTurnDistance && pp.getX() >= 0) {
                // difference between x less than two turn
                return RelativeDirection.FRONT_SLIGHT_RIGHT;
            } else if (pp.getX() >= -twoTurnDistance && pp.getX() <= 0) {
                return RelativeDirection.FRONT_SLIGHT_LEFT;
            } else if (pp.getX() >= 0) {
                return RelativeDirection.FRONT_RIGHT;
            } else if (pp.getX() <= 0) {
                return RelativeDirection.FRONT_LEFT;
            }
        } else if (twoTurnDistance <= pp.getY()) {
            // point is behind
            if (pp.getX() <= twoTurnDistance && pp.getX() >= 0) {
                // difference between x less than two turn
                return RelativeDirection.BACK_SLIGHT_RIGHT;
            } else if (pp.getX() >= -twoTurnDistance && pp.getX() <= 0) {
                return RelativeDirection.BACK_SLIGHT_LEFT;
            } else if (pp.getX() >= 0) {
                return RelativeDirection.BACK_RIGHT;
            } else if (pp.getX() <= 0) {
                return RelativeDirection.BACK_LEFT;
            }
        }

        return null;
    }

    public Direction getRelativeOrientation(MyPoint p1, MyPoint p2) {
        // compare two points directions and from p2's relative orientation to p1
        // meaning, assuming from p1's POV of facing forward = NORTH
        int a = p2.getDirection().ordinal() - p1.getDirection().ordinal();
        switch (a) {
        case 0:
            return Direction.NORTH;
        case 1:
        case -3:
            return Direction.EAST;
        case 2:
        case -2:
            return Direction.SOUTH;
        case 3:
        case -1:
            return Direction.WEST;
        }
        return null;
    }

    public double getSpeed() {
        return this.speed;
    }

    public double getTimeFor90DegTurn() {
        // not really working properly
        return 2.26875 * 0.25 * 2 * Math.PI * this.MAX_TURNING_RADIUS / this.speed;
    }

    public double getDurationForManeuver(double dist) {
        return (dist / this.getSpeed());
    }

    double getEuclideanDistance(Point a, Point b) {
        return Math.sqrt(Math.pow(Math.abs(a.getX() - b.getX()), 2) + Math.pow(Math.abs(a.getY() - b.getY()), 2));
    }

    public void generateMovements(ArrayList<MyPoint> plannedPath) {
        this.movementQueue.clear();
        this.durationQueue.clear();
        this.directionQueue.clear();
        boolean justTurned = false;
        double dist;
        for (int i = 1; i < plannedPath.size(); i++) {
            MyPoint src = plannedPath.get(i - 1);
            MyPoint dest = plannedPath.get(i);
            // System.out.println("src: " + src.x + " " + src.y);
            // System.out.println("dest: " + dest.x + " " + dest.y);
            // System.out.println(this.getRelativeDirection(src, dest));
            switch (this.getRelativeOrientation(src, dest)) {
            case NORTH:
                dist = this.getEuclideanDistance(src, dest);
                if (justTurned) {
                    dist -= this.getTwoTurnsDistance() / 2;
                    justTurned = false;
                }
                switch (this.getRelativeDirection(src, dest)) {
                case BACK:
                    this.addToQueue("Reverse", this.getDurationForManeuver(dist), Direction.NONE);
                    justTurned = false;
                    break;
                case FRONT:
                    this.addToQueue("Forward", this.getDurationForManeuver(dist), Direction.NONE);
                    justTurned = false;
                    break;
                case NONE:
                    break;
                default:
                    break;
                }
                break;
            case SOUTH:
                // dist = this.getEuclideanDistance(src, dest);
                // this.addToQueue("Reverse", this.getDurationForManeuver(dist));
                break;
            case EAST:
                dist = this.getEuclideanDistance(src, dest);
                dist -= this.getTwoTurnsDistance() / 2;
                if (justTurned) {
                    dist -= this.getTwoTurnsDistance() / 2;
                    justTurned = false;
                }
                this.addToQueue("Forward", this.getDurationForManeuver(dist), Direction.NONE);
                this.addToQueue("Right", 1, Direction.NONE);
                this.addToQueue("RF", 1000, dest.getDirection());
                this.addToQueue("Center", 1, Direction.NONE);
                justTurned = true;
                break;
            case WEST:
                dist = this.getEuclideanDistance(src, dest);
                dist -= this.getTwoTurnsDistance() / 2;
                if (justTurned) {
                    dist -= this.getTwoTurnsDistance() / 2;
                    justTurned = false;
                }
                this.addToQueue("Forward", this.getDurationForManeuver(dist), Direction.NONE);
                this.addToQueue("Left", 1, Direction.NONE);
                this.addToQueue("LF", 1000, dest.getDirection());
                this.addToQueue("Center", 1, Direction.NONE);
                justTurned = true;
                break;
            case NONE:
                break;
            default:
                break;

            }

        }
    }

    public void printGeneratedMovements() {
        for (int i = 0; i < this.movementQueue.size(); i++) {
            System.out.println(
                    this.movementQueue.get(i) + " " + this.durationQueue.get(i) + " " + this.directionQueue.get(i));
        }
    }
}