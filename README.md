### To use

1. Locate the _project root directory,_ whose path ends with `Algorithm`.
2. Type `javac -cp .\lib\*;.\src\main\java -Xlint:none -d .\bin .\src\main\java\simulator\Simulator.java` and press
   enter to compile the code.
3. Then, type `java -classpath .\lib\*;.\bin simulator.Simulator` and press enter to run the program.

### Details to note

The environment:

- Physically will be 200cm x 200cm
- Grid is represented with a 20x20 grid of cells
- Each cell represents 10cm x 10cm

The "robot" model:

- Will be modeled as a 3x3 on the grid
- Physical size is closer to a 2x2 size on the grid (20cm x 21cm)
- Turning radius:
    - The documents provided gives it as around 25cm turning radius

The "obstacle" model:

- Physical size is identical to size on grid (1x1) (10cm x 10cm)

###