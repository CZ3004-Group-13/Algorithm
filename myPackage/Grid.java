package myPackage;

import java.awt.*;
import java.util.ArrayList;
import javax.swing.*;

public class Grid extends JPanel {

  private Cell[][] cells = new Cell[20][20];

  public Grid() {
    setBorder(BorderFactory.createLineBorder(Color.black));
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
    ArrayList<Point> obstacleCenters = new ArrayList<Point>();
    obstacleCenters.clear();

    Dimension size = cells[0][0].getSize();

    for (Cell[] cells2 : cells) {
      for (Cell cell : cells2) {
        if (cell.isObstaclePresent()) {
          obstacleCenters.add(new Point(cell.getX()+size.width/2, cell.getY()+size.height/2));
        }
      }
    }

    Point[] array = new Point[obstacleCenters.size()];
    array = obstacleCenters.toArray(array);

    return array;
  }
}
