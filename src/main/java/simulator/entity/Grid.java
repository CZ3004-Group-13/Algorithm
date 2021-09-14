package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.awt.geom.*;

public class Grid extends JPanel {

    private final Cell[][] cells = new Cell[20][20];
    private Dimension envModelSize;
    private Dimension cellModelSize;
    private double boundaryLength;

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
        this.boundaryLength = cellModelSize.getWidth() * 1.2;
    }

    public MyPoint[] getObstacleFronts() {
        ArrayList<MyPoint> obstacleFront = new ArrayList<>();

        Dimension size = cells[0][0].getSize();

        int offset = (int) this.boundaryLength + 5 + size.width / 2;

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
        return checkIfExceedsBorders(p);
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
        return checkIfExceedsBorders(p1) || checkIfExceedsBorders(p2);
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
                p.translate(0, (int) -turningRadius);
                return this.checkIfPointCollides(p);
            case SOUTH:
                p.translate(0, (int) turningRadius);
                return this.checkIfPointCollides(p);
            case EAST:
                p.translate((int) turningRadius, 0);
                return this.checkIfPointCollides(p);
            case WEST:
                p.translate((int) -turningRadius, 0);
                return this.checkIfPointCollides(p);
            default:
                break;

        }

        return false;
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
