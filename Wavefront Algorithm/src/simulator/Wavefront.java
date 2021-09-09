package simulator;
 
import java.awt.Dimension;
import java.util.ArrayList;
 

public class Wavefront {
    private final int gridsizeX;
    private final int gridsizeY;
    private int[][] cell;
    private int target;
    private int dirChange = 0;
    private IUICallback ui;
 
    private ArrayList<Dimension> pos = new ArrayList<Dimension>();
 
    private static int GOAL = 3;
    private static int START = 2;
    private static int FREE = 0;
    private static int WALL = 1;
 
    private static int MARKER1 = 1;
 
 
    public Wavefront(int[][] cell, IUICallback ui) {
        this.cell = cell;
        this.ui = ui;
 
        target = GOAL;
        this.gridsizeX = cell[0].length;
        this.gridsizeY = cell.length;
    }
 
    public int autoRun() {
        int rc = 0;
        try {
            while(!wavefrontPass()) {
            }
        } catch(EndlessLoopException ele) {
            rc = 1;         
        } catch(NoPathFoundException npf) {
            rc = 2;
        } 
        return rc;
    }


    public boolean wavefrontPass() throws EndlessLoopException, NoPathFoundException {
        boolean found = false;
        boolean finished = false;
 
        target++;
        if(target > cell.length * cell[0].length) {
            throw new EndlessLoopException("An endless loop has been detected. Target value is: " + target);
        }
        for (int row = 0; row < cell.length; row++) {
            for (int col = 0; col < cell[row].length; col++) {
                // Only unoccupied cells need to be checked for neighbors
                if(cell[row][col] == 0) {
                    // Watch out for a cell with target - 1 value
                    int tmpProgress = findNeighbor(row, col, target-1);
                    if(tmpProgress > 8) {
                        // Found the start. Progress indicates the direction where it was found.
                        // Store cell coordinates which was identified as neighbor of "S".
                        pos.add(new Dimension(row, col));
                        finished = true;
                    }
                    // At least a neighbor cell was found
                    if(tmpProgress > 0) {
                        found = true;
                        cell[row][col] = target;
                        // The callback to the simulator applications UI update
                        if(ui != null) ui.updateTileLabel(row, col, target);
                    }
                }
            }
        }
        if(!found) {
            throw new NoPathFoundException("No path can be found. Target value is: " + target);
        }
        return finished;
    }
 
    private int findNeighbor(int row, int col, int prevTarget) {
        int result = 0;
        if((row - 1) > 0 && cell[row-1][col] == prevTarget) {
            result = 1; //Top
        } else if((col + 1) < gridsizeX && cell[row][col+1] == prevTarget) {
            result = 2; //Right
        } else if((row + 1) < gridsizeY && cell[row+1][col] == prevTarget) {
            result = 3; //Bottom
        } else if((col - 1) > 0 && cell[row][col-1] == prevTarget) {
            result = 4; //Left
        } else if (((row - 1) > 0) && ((col - 1) > 0) && (cell[row-1][col-1] == prevTarget)) {
        	result = 5; //Top Left
        } else if(((row - 1) > 0) && ((col + 1) < gridsizeX) && (cell[row-1][col+1] == prevTarget)) {
        	result = 6; //Top Right
        } else if(((row + 1) < gridsizeY) && ((col + 1) < gridsizeX) && (cell[row+1][col+1] == prevTarget)) {
        	result = 7; //Bottom Right
        } else if (((row + 1) < gridsizeY) && ((col - 1) > 0) && (cell[row+1][col-1] == prevTarget)) {
        	result = 8; //Bottom Left
        } 
        // Find out if the next target tile also neighbors the car tile
        if(result > 0) {
            int x = checkPathFound(row , col);
            if(x > 0) {
                result = x;
            }
        }
        return result;
    }
 
    private int checkPathFound(int row, int col) {
        int result = 0;
        if(cell[row-1][col] == START) {
            result = 9; //Top
        } else if(cell[row][col+1] == START) {
            result = 10; //Right
        } else if(cell[row+1][col] == START) {
            result = 11; //Bottom
        } else if(cell[row][col-1] == START) {
            result = 12; //Left
        } else if(cell[row-1][col-1] == START) {
            result = 13; //Top Left
        } else if(cell[row-1][col+1] == START) {
            result = 14; //Top Right
        } else if(cell[row+1][col+1] == START) {
            result = 15; //Bottom Right
        } else if(cell[row+1][col-1] == START) {
            result = 16; //Bottom Left
        }
        return result;
    }
 
    /*
     * Follow the path backwards while trying to maintain the same direction
     */
    private void recursive(final int row, final int col, final int target, final int direction, final boolean markUICell) {
        int ii = 0;
        int jj = 0;
 
        if(markUICell) if(ui != null) ui.paintTile(row, col, MARKER1);
        // Determine the von Neumann direction ...
        switch (direction) {
        case 1: //Top
            ii = -1; jj = 0;
            break;
        case 2: //Right
            ii = 0; jj = 1;
            break;
        case 3: //Bottom
            ii = 1; jj = 0;
            break;
        case 4: //Left
            ii = 0; jj = -1;
            break;
        case 5: //Top Left
        	ii = -1; jj = -1;
        case 6: //Top Right
        	ii = -1; jj = 1;
        case 7: //Bottom Right
        	ii = 1; jj = 1;
        case 8: //Bottom Left
        	ii = 1; jj = -1;
        }
        // ... and check if the next lower number (t-1) can be found in this direction.
        if(cell[row+ii][col+jj] == target - 1) {
            recursive(row+ii, col+jj, target-1, direction, markUICell);
            // If the next lower number could not be found in the stored direction, try all other directions
        } else {
            dirChange++;
            if(cell[row-1][col] == target - 1) {
                recursive(row-1, col, target-1, 1, markUICell); //Top
            } else if(cell[row][col+1] == target - 1) {
                recursive(row, col+1, target-1, 2, markUICell); //Right
            } else if(cell[row+1][col] == target - 1) {
                recursive(row+1, col, target-1, 3, markUICell); //Bottom
            } else if(cell[row][col-1] == target - 1) {
                recursive(row, col-1, target-1, 4, markUICell); //Left
            } else if(cell[row-1][col-1] == target - 1) {
                recursive(row-1, col-1, target-1, 5, markUICell); //Top Left
            } else if(cell[row-1][col+1] == target - 1) {
                recursive(row-1, col+1, target-1, 6, markUICell); //Top Right
            } else if(cell[row+1][col+1] == target - 1) {
                recursive(row+1, col+1, target-1, 7, markUICell); //Bottom Right
            } else if(cell[row+1][col-1] == target - 1) {
                recursive(row+1, col-1, target-1, 8, markUICell); //Bottom Left
            }
        }
        return;
    }
 
 
    /**
     * Returns a List with the sequence of cell coordinates that make up the path between 
     * Start and Goal
     * 
     * @return  ArrayList<Dimension>
     */
    public ArrayList<Dimension> getPath() {
        int tmpPos = 0;
        int tmpDirChanges = Integer.MAX_VALUE;
        ArrayList<Dimension> path = new ArrayList<Dimension>();
 
        for(int a=0; a<pos.size(); a++) {
            // Find initial direction from goal to first adjacent number 
            int direction = checkPathFound(pos.get(a).width, pos.get(a).height) - 8;
            // Get the number of directional changes
            dirChange = 0;
            recursive(pos.get(a).width, pos.get(a).height, target, direction, false);
            // If smaller than previous paths ...
            if(dirChange < tmpDirChanges) { 
                // Store number ...
                tmpDirChanges = dirChange;
                // ... and source cell from which the path originates
                tmpPos = a;
            }
        }
        //Finally, the optimal path will be marked
        recursive(pos.get(tmpPos).width, pos.get(tmpPos).height, target,
                  checkPathFound(pos.get(tmpPos).width, pos.get(tmpPos).height) - 8, true);
        return path;
    }
 
    @SuppressWarnings("serial")
    class EndlessLoopException extends Exception
    {
        // Parameterless Constructor
        public EndlessLoopException() {}
 
        // Constructor that accepts a message
        public EndlessLoopException(String message)
        {
            super(message);
        }
    }
 
    @SuppressWarnings("serial")
    class NoPathFoundException extends Exception
    {
        // Parameterless Constructor
        public NoPathFoundException() {}
 
        // Constructor that accepts a message
        public NoPathFoundException(String message)
        {
            super(message);
        }
    }
}