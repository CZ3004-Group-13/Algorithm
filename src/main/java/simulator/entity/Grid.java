package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.awt.geom.*;

public class Grid extends JPanel {

    private final Cell[][] cells = new Cell[20][20];

    public Grid() {
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
    }

    public MyPoint[] getObstacleFronts() {
        ArrayList<MyPoint> obstacleFront = new ArrayList<>();

        Dimension size = cells[0][0].getSize();

        int offset = 50;

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

        Dimension size = new Dimension((int) (cells[0][0].getSize().width * 4),
                (int) (cells[0][0].getSize().height * 4));
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
