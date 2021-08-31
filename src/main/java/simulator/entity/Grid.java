package simulator.entity;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

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

    public Point[] getObstacleCenters() {
        ArrayList<Point> obstacleCenters = new ArrayList<>();

        Dimension size = cells[0][0].getSize();

        for (Cell[] row : cells) {
            for (Cell cell : row) {
                if (cell.isObstaclePresent()) {
                    obstacleCenters.add(new Point(cell.getX() + size.width / 2, cell.getY() + size.height / 2));
                }
            }
        }

        return obstacleCenters.toArray(new Point[0]);
    }
}
