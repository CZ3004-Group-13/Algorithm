
package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.NoninvertibleTransformException;
import java.awt.geom.Rectangle2D;
import java.awt.geom.Point2D;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Robot extends JComponent {
    private final Rectangle2D body;
    private final Rectangle2D leftWheel;
    private final Rectangle2D rightWheel;
    private final Dimension size;
    private final double distanceBetweenFrontBackWheels;
    private Point startingPoint;
    private Point currentLocation;
    private Point currentLocationCenter;
    private AffineTransform bodyAffineTransform = new AffineTransform();
    private AffineTransform leftWheelAffineTransform = new AffineTransform();
    private AffineTransform rightWheelAffineTransform = new AffineTransform();

    private double speed;
    private double directionInDegrees = -90;
    private final double distancePerTick; // the distance the robot moves per tick
    private final double angleChangePerClick;
    // need to incorporate concept of speed later
    private double turningRadius = 0;
    private double turningAngleDegrees = 0;
    private static final double THETA_CAR = 270; // this is facing up
    private double thetaWheelsDegree = 0; // with reference to the front of car
    // since the coordinate system is positive x is right, positive y is down,
    // angles work by starting from positive x and going clockwise

    private static final double DIRECTION_MARGIN_OF_ERROR = 1.2;
    private final double MAX_TURNING_ANGLE = 45;
    public final double MAX_TURNING_RADIUS;

    private int avoidingPhase = 0;
    private Direction directionToTurnFrom = Direction.NONE;
    private Direction directionToTurn = Direction.NONE;
    private Rectangle2D avoidingObstacle;

    private final Logger logger;

    private boolean firstTime = true;

    public Robot(Dimension size, Point startingPoint, double distanceBetweenFrontBackWheels) {
        logger = Logger.getLogger(Robot.class.getName());

        this.size = size;
        this.startingPoint = startingPoint;
        this.setCurrentLocationCenter(startingPoint);
        body = new Rectangle2D.Double(0, 0, size.getWidth(), size.getHeight());
        leftWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);
        rightWheel = new Rectangle2D.Double(0, 0, size.getWidth() / 5, size.getHeight() / 3);

        this.distanceBetweenFrontBackWheels = distanceBetweenFrontBackWheels;
        distancePerTick = 1;
        angleChangePerClick = 5;

        this.MAX_TURNING_RADIUS = Math.abs(
                this.distanceBetweenFrontBackWheels * Math.tan(Math.PI / 2 - Math.toRadians(this.MAX_TURNING_ANGLE)));

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
        return MAX_TURNING_RADIUS * 2.5;
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

        // 0 is detection
        // 1 is first turn
        // 2 is second turn to go straight again
        // 3 is going straight and detecting when to turn again
        // 4 is third turn
        // 5 is fourth turn
        // if (avoidingPhase == 0) {
        // moveForward();
        // directionToTurnFrom = this.getGeneralDirection();
        // double x = 0, y = 0;
        // switch (directionToTurnFrom) {
        // case NORTH:
        // x = 0;
        // y = -this.MAX_TURNING_RADIUS;
        // break;
        // case SOUTH:
        // x = 0;
        // y = this.MAX_TURNING_RADIUS;
        // break;
        // case EAST:
        // x = this.MAX_TURNING_RADIUS;
        // y = 0;
        // break;
        // case WEST:
        // x = -this.MAX_TURNING_RADIUS;
        // y = 0;
        // break;
        // case NONE:
        // break;

        // }

        // for (Rectangle2D o : obstacles) {
        // MyPoint p = this.getCurrentLocation();
        // p.translate((int) x, (int) y);

        // if (o.contains(p) && !o.contains(myPoint)) {
        // System.out.println("Colliding");
        // this.avoidingObstacle = o;
        // this.avoidingPhase = 1;
        // switch (directionToTurnFrom) {
        // case NORTH:
        // if (p.getX() > o.getCenterX()) {
        // directionToTurn = Direction.EAST;
        // } else {
        // directionToTurn = Direction.WEST;
        // }
        // break;
        // case SOUTH:
        // if (p.getX() > o.getCenterX()) {
        // directionToTurn = Direction.EAST;
        // } else {
        // directionToTurn = Direction.WEST;
        // }
        // break;
        // case EAST:
        // if (p.getY() > o.getCenterY()) {
        // directionToTurn = Direction.SOUTH;
        // } else {
        // directionToTurn = Direction.NORTH;
        // }
        // break;
        // case WEST:
        // if (p.getY() > o.getCenterY()) {
        // directionToTurn = Direction.SOUTH;
        // } else {
        // directionToTurn = Direction.NORTH;
        // }
        // break;
        // case NONE:
        // default:
        // directionToTurn = Direction.NONE;
        // break;

        // }
        // }
        // }
        // } else if (this.avoidingPhase == 1
        // || this.avoidingPhase == 2 | this.avoidingPhase == 4 | this.avoidingPhase ==
        // 5) {
        // boolean turned = false;
        // switch (directionToTurnFrom) {
        // case NORTH:
        // if (directionToTurn == Direction.EAST) {
        // turned = this.moveForwardRightWithChecking(directionToTurn);
        // } else if (directionToTurn == Direction.WEST) {
        // turned = this.moveForwardLeftWithChecking(directionToTurn);
        // }
        // break;
        // case SOUTH:
        // if (directionToTurn == Direction.EAST) {
        // turned = this.moveForwardLeftWithChecking(directionToTurn);
        // } else if (directionToTurn == Direction.WEST) {
        // turned = this.moveForwardRightWithChecking(directionToTurn);
        // }
        // break;
        // case EAST:
        // if (directionToTurn == Direction.SOUTH) {
        // turned = this.moveForwardRightWithChecking(directionToTurn);
        // } else if (directionToTurn == Direction.NORTH) {
        // turned = this.moveForwardLeftWithChecking(directionToTurn);
        // }
        // break;
        // case WEST:
        // if (directionToTurn == Direction.SOUTH) {
        // turned = this.moveForwardLeftWithChecking(directionToTurn);
        // } else if (directionToTurn == Direction.NORTH) {
        // turned = this.moveForwardRightWithChecking(directionToTurn);
        // }
        // break;
        // case NONE:
        // default:
        // directionToTurn = Direction.NONE;
        // break;

        // }
        // if (turned) {
        // if (this.avoidingPhase == 1) {
        // Direction temp = directionToTurnFrom;
        // directionToTurnFrom = directionToTurn;
        // directionToTurn = temp;
        // avoidingPhase = 2;
        // } else if (this.avoidingPhase == 2) {
        // avoidingPhase = 3;
        // } else if (this.avoidingPhase == 4) {
        // Direction temp = directionToTurnFrom;
        // directionToTurnFrom = directionToTurn;
        // directionToTurn = temp;
        // avoidingPhase = 5;
        // } else if (this.avoidingPhase == 5) {
        // this.avoidingPhase = 0;
        // this.directionToTurn = Direction.NONE;
        // this.directionToTurnFrom = Direction.NONE;
        // this.avoidingObstacle = null;
        // }
        // }
        // } else if (this.avoidingPhase == 3) {
        // // calculate a relative point from the robot's center,
        // // to see when that relative point no longer will collide
        // moveForward();
        // double x = 0, y = 0;
        // switch (directionToTurnFrom) {
        // case NORTH:
        // x = 0;
        // y = this.MAX_TURNING_RADIUS * 2;
        // break;
        // case SOUTH:
        // x = 0;
        // y = -this.MAX_TURNING_RADIUS * 2;
        // break;
        // case EAST:
        // x = -this.MAX_TURNING_RADIUS * 2;
        // y = 0;
        // break;
        // case WEST:
        // x = this.MAX_TURNING_RADIUS * 2;
        // y = 0;
        // break;
        // case NONE:
        // break;
        // }

        // MyPoint p = this.getCurrentLocation();
        // p.translate((int) x, (int) y);

        // if (!avoidingObstacle.contains(p)) {
        // this.avoidingPhase = 4;
        // Direction temp = directionToTurn;
        // switch (directionToTurnFrom) {
        // case NORTH:
        // directionToTurn = Direction.SOUTH;
        // break;
        // case SOUTH:
        // directionToTurn = Direction.NORTH;
        // break;
        // case EAST:
        // directionToTurn = Direction.WEST;
        // break;
        // case WEST:
        // directionToTurn = Direction.EAST;
        // break;
        // default:
        // break;
        // }
        // directionToTurnFrom = temp;
        // }
        // } else {
        moveForward();
        // }
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

    // calculate turningRadius and turningAngleDegrees
    // based on distance between front and back wheel axis
    private void doWheelsTurned() {
        this.turningRadius = Math.abs(
                this.distanceBetweenFrontBackWheels * Math.tan(Math.PI / 2 - Math.toRadians(this.thetaWheelsDegree)));

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
        pp.translate((int) -p1.getX(), (int) -p1.getY());

        AffineTransform af = new AffineTransform();
        af.setToIdentity();
        switch (p1.getDirection()) {
            case NORTH:
                af.rotate(Math.toRadians(0));
                break;
            case SOUTH:
                af.rotate(Math.toRadians(180));
                break;
            case EAST:
                af.rotate(Math.toRadians(-90));
                break;
            case WEST:
                af.rotate(Math.toRadians(90));
                break;
            case NONE:
                break;
            default:
                break;

        }

        af.transform(pp, pp);

        if (-this.size.width / 2 <= pp.getX() && this.size.width / 2 >= pp.getX()) {
            if (pp.getY() <= 0) {
                return RelativeDirection.FRONT;
            } else if (pp.getY() >= 0) {
                return RelativeDirection.BACK;
            }
        } else if (-this.getTwoTurnsDistance() >= pp.getY()) {
            // point in front within two turn margin
            if (pp.getX() <= this.getTwoTurnsDistance() && pp.getX() >= 0) {
                // difference between x less than two turn
                return RelativeDirection.FRONT_SLIGHT_RIGHT;
            } else if (pp.getX() >= -this.getTwoTurnsDistance() && pp.getX() <= 0) {
                return RelativeDirection.FRONT_SLIGHT_LEFT;
            } else if (pp.getX() >= 0) {
                return RelativeDirection.FRONT_RIGHT;
            } else if (pp.getX() <= 0) {
                return RelativeDirection.FRONT_LEFT;
            }
        } else if (this.getTwoTurnsDistance() <= pp.getY()) {
            // point is behind
            if (pp.getX() <= this.getTwoTurnsDistance() && pp.getX() >= 0) {
                // difference between x less than two turn
                return RelativeDirection.BACK_SLIGHT_RIGHT;
            } else if (pp.getX() >= -this.getTwoTurnsDistance() && pp.getX() <= 0) {
                return RelativeDirection.BACK_SLIGHT_LEFT;
            } else if (pp.getX() >= 0) {
                return RelativeDirection.BACK_RIGHT;
            } else if (pp.getX() <= 0) {
                return RelativeDirection.BACK_LEFT;
            }
        } else if (0 <= pp.getY() + this.getTwoTurnsDistance()) {
            // within the 2 turn margin of front and back
            if (0 <= pp.getX()) {
                return RelativeDirection.CENTER_RIGHT;
            } else if (0 >= pp.getX()) {
                return RelativeDirection.CENTER_LEFT;
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
}