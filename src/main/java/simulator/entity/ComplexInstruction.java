package simulator.entity;


public class ComplexInstruction {

    private Direction finalDirection;
    private Instruction instruction;

    public Instruction getInstruction() {
        return instruction;
    }

    public Direction getFinalDirection() {
        return finalDirection;
    }

    public enum Instruction {
        FORWARD,
        REVERSE,
        FORWARD_LEFT,
        FORWARD_RIGHT,
    }

    public ComplexInstruction(Instruction instruction, Direction finalDirection) {
        this.instruction = instruction;
        this.finalDirection = finalDirection;

    }

}
