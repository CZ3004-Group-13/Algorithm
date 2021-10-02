package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;

public class Grid extends JPanel {

    private final Cell[][] cells = new Cell[20][20];
    private Dimension envModelSize;
    private Dimension cellModelSize;
    private double boundaryLength;
    private boolean allowOutOfBounds = true;

    public Grid(Dimension envModelSize) {
        // setBorder(BorderFactory.createLineBorder(Color.black));
        setLayout(new GridLayout(20, 20));

        Dimension cellDimension = new Dimension(10, 10); // this is really small but the GridLayout will scale it up

        for (int i = 0; i < 20; i++) {
            for (int j = 0; j < 20; j++) {
                cells[i][j] = new Cell();
                cells[i][j].setSize(cellDimension);
                // cells[i][j].setPreferredSize(cellDimension);
                // cells[i][j].setMaximumSize(cellDimension);
                add(cells[i][j]);
            }
        }
        this.envModelSize = envModelSize;
        this.cellModelSize = new Dimension((int) envModelSize.getWidth() / 20, (int) envModelSize.getHeight() / 20);
        // this.boundaryLength = cellModelSize.getWidth() * 1.5;
        this.boundaryLength = 30 + 15;
    }

    public void addObstacle(int x, int y, int z) {
        Direction d = Direction.NONE;
        switch (z) {
            case 0:
                d = Direction.NORTH;
                break;
            case 1:
                d = Direction.EAST;
                break;
            case 2:
                d = Direction.SOUTH;
                break;
            case 3:
                d = Direction.WEST;
                break;
            default:
                break;
        }
        cells[x][y].setObstaclePresent(true);
        cells[x][y].setImageDirection(d);
        this.repaint();
    }

    public MyPoint[] getObstacleFronts() {
        ArrayList<MyPoint> obstacleFront = new ArrayList<>();

        Dimension size = cells[0][0].getSize();

        int offset = (int) this.boundaryLength + size.width / 2 + 25;

        for (Cell[] row : cells) {
            for (Cell cell : row) {
                if (cell.isObstaclePresent()) {
                    switch (cell.getImageDirection()) {
                        case NONE:
                            break;
                        case NORTH:
                            obstacleFront.add(new MyPoint(cell.getX() + size.width / 2,
                                    cell.getY() + size.height / 2 - offset, cell.getImageDirection()));
                            break;
                        case SOUTH:
                            obstacleFront.add(new MyPoint(cell.getX() + size.width / 2,
                                    cell.getY() + size.height / 2 + offset, cell.getImageDirection()));
                            break;
                        case EAST:
                            obstacleFront.add(new MyPoint(cell.getX() + size.width / 2 + offset,
                                    cell.getY() + size.height / 2, cell.getImageDirection()));
                            break;
                        case WEST:
                            obstacleFront.add(new MyPoint(cell.getX() + size.width / 2 - offset,
                                    cell.getY() + size.height / 2, cell.getImageDirection()));
                            break;
                        default:
                            break;

                    }

                }
            }
        }

        return obstacleFront.toArray(new MyPoint[0]);
    }

    public Rectangle2D[] getObstacleBoundaries() {
        ArrayList<Rectangle2D> obstacleBoundaries = new ArrayList<>();

        Dimension size = new Dimension((int) (cells[0][0].getSize().width + 2 * this.boundaryLength),
                (int) (cells[0][0].getSize().height + 2 * this.boundaryLength));
        // obstacle size 10x10, robot size 20x20, so
        // buffer of 15 is enough?
        // 40 x 40
        for (Cell[] row : cells) {
            for (Cell cell : row) {
                if (cell.isObstaclePresent()) {
                    obstacleBoundaries.add(
                            new Rectangle2D.Double(cell.getX() + cell.getSize().getWidth() / 2 - size.getWidth() / 2,
                                    cell.getY() + cell.getSize().getHeight() / 2 - size.getHeight() / 2,
                                    size.getWidth(), size.getHeight()));
                }
            }
        }
        return obstacleBoundaries.toArray(new Rectangle2D[0]);
    }

    public boolean checkIfPointCollides(Point p) {
        Rectangle2D[] obs = this.getObstacleBoundaries();

        for (Rectangle2D o : obs) {
            if (o.contains(p)) {
                return true;
            }
        }
        if (allowOutOfBounds) {
            return false;
        } else {
            return checkIfExceedsBorders(p);
        }
    }

    public boolean checkIfLineCollides(Point p1, Point p2) {
        Rectangle2D[] obs = this.getObstacleBoundaries();
        Line2D line = new Line2D.Double(p1, p2);

        for (Rectangle2D o : obs) {
            if (o.intersectsLine(line)) {
                // System.out.println("Line collide:");
                // System.out.println("p1 " + p1.x + " " + p1.y);
                // System.out.println("p2 " + p2.x + " " + p2.y);
                return true;
            }
        }
        if (allowOutOfBounds) {
            return false;
        } else {
            return checkIfExceedsBorders(p1) || checkIfExceedsBorders(p2);
        }
    }

    public boolean checkIfPathCollides(Point[] pArray) {
        boolean flag = false;
        for (int i = 0; i < pArray.length - 1; i++) {
            flag = flag || this.checkIfLineCollides(pArray[i], pArray[i + 1]);
        }

        return flag;
    }

    private boolean checkIfExceedsBorders(Point p) {
        // since p will be center of robot, need give boundary buffer
        if (p.getX() < 0 + this.boundaryLength || p.getY() < 0 + this.boundaryLength
                || p.getX() > this.envModelSize.getWidth() - this.boundaryLength
                || p.getY() > this.envModelSize.getHeight() - this.boundaryLength) {
            System.out.println("Out of bounds detected");
            return true;
        } else {
            return false;
        }
    }

    public boolean checkIfNeedReverse(MyPoint p, int turningRadius) {
        p = (MyPoint) p.clone();
        switch (p.getDirection()) {
            case NONE:
                break;
            case NORTH:
                p.translate(0, -turningRadius);
                return this.checkIfPointCollides(p);
            case SOUTH:
                p.translate(0, turningRadius);
                return this.checkIfPointCollides(p);
            case EAST:
                p.translate(turningRadius, 0);
                return this.checkIfPointCollides(p);
            case WEST:
                p.translate(-turningRadius, 0);
                return this.checkIfPointCollides(p);
            default:
                break;

        }

        return false;
    }

    public double getAmountToReverse(MyPoint p, int turningRadius) {
        Rectangle2D[] obs = this.getObstacleBoundaries();

        Line2D line = null;
        MyPoint p2 = (MyPoint) p.clone();

        switch (p.getDirection()) {
            case NONE:
                break;
            case NORTH:
                p2.translate(0, -turningRadius);
                line = new Line2D.Double(p, p2);
                for (Rectangle2D o : obs) {
                    if (o.intersectsLine(line)) {

                        double centerToBoundary = this.cellModelSize.getHeight() / 2 + this.boundaryLength;
                        double diff = Math.abs(p.getY() - o.getCenterY());
                        double amt = turningRadius - (diff - centerToBoundary);
                        return amt;
                    }
                }
            case SOUTH:
                p2.translate(0, turningRadius);
                line = new Line2D.Double(p, p2);
                for (Rectangle2D o : obs) {
                    if (o.intersectsLine(line)) {

                        double centerToBoundary = this.cellModelSize.getHeight() / 2 + this.boundaryLength;
                        double diff = Math.abs(p.getY() - o.getCenterY());
                        double amt = turningRadius - (diff - centerToBoundary);
                        return amt;
                    }
                }
            case EAST:
                p2.translate(-turningRadius, 0);
                line = new Line2D.Double(p, p2);
                for (Rectangle2D o : obs) {
                    if (o.intersectsLine(line)) {
                        double centerToBoundary = this.cellModelSize.getHeight() / 2 + this.boundaryLength;
                        double diff = Math.abs(p.getX() - o.getCenterX());
                        double amt = turningRadius - (diff - centerToBoundary);
                        return amt;
                    }
                }
            case WEST:
                p2.translate(turningRadius, 0);
                line = new Line2D.Double(p, p2);
                for (Rectangle2D o : obs) {
                    if (o.intersectsLine(line)) {
                        double centerToBoundary = this.cellModelSize.getHeight() / 2 + this.boundaryLength;
                        double diff = Math.abs(p.getX() - o.getCenterX());
                        double amt = turningRadius - (diff - centerToBoundary);
                        return amt;
                    }
                }
            default:
                break;

        }
        return 0.0;
    }

    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2 = (Graphics2D) g;
        g2.setColor(Color.cyan);

        if (true) {
            Rectangle2D[] boundaries = this.getObstacleBoundaries();
            for (Rectangle2D b : boundaries) {
                g2.draw(b);
            }
        }
    }
}
