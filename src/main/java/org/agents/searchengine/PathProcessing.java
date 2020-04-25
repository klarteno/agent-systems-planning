package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.Color;
import org.agents.MapFixedObjects;
import org.agents.action.Direction;

import java.util.*;
import java.util.stream.Stream;

public class PathProcessing {
    public static final String MoveAction = "Move";
    public static final String PushAction = "Push";
    public static final String PullAction = "Pull";

    MapFixedObjects mapFixedObjects;

    public  PathProcessing() {
        mapFixedObjects = new MapFixedObjects();
    }

    private ArrayList<String> getMoves(Stack<int[]> path){
        assert path!=null;
        ArrayList<String> agent_moves = new ArrayList<>();
        int[] next_cell = path.pop();
        while (!path.empty()){
            int[] prev_cell = path.pop();
            Direction next_direction = Direction.getDirectionsFrom(prev_cell, next_cell);
            agent_moves.add(MoveAction + "(" + next_direction.toString() + ")" );
            next_cell = prev_cell;
        }
        return agent_moves;
    }

    private ArrayList<String> getBoxMoves(Agent agent, int[] agent_cell, Stack<int[]> box_path){
        assert agent_cell != null;

        ArrayList<String> agent_moves = new ArrayList<>();

        Iterator<int[]> iter = box_path.iterator();
        int[] prev_cell = iter.next();
        int[] next_cell;

        while(iter.hasNext()){
            next_cell = iter.next();
            Direction position = Direction.getDirectionsFrom(agent_cell, prev_cell);
            if(Arrays.equals(next_cell,agent_cell)){
                Stack<int[]> cells = mapFixedObjects.getNeighbours(agent_cell, agent.getColor());
                Stream<int[]> next_neighbours = cells.stream().filter(box_path::contains);//to optimize and abstract
                int[] cell = this.selectCellForAgent(next_neighbours);
                Direction agent_dir = Direction.getDirectionsFrom(agent_cell, cell);
                agent_moves.add(PullAction + "(" + agent_dir.toString() + "," + position.toString() +")" );
                agent_dir.getNextCellFrom(agent_cell);
            }else {
                Direction next_box_dir = Direction.getDirectionsFrom(prev_cell, next_cell);
                agent_moves.add(PushAction + "(" + position.toString() + "," + next_box_dir.toString() +")" );

                agent_cell = prev_cell;
            }
            prev_cell = next_cell;
        }

        return agent_moves;
    }

    private int[] selectCellForAgent(Stream<int[]> next_neighbours) {
        Optional<int[]> any = next_neighbours.findAny();
        if (any.isPresent()) {
            return any.get();
        }
        else {
            System.out.println("no value");
            return new int[]{-2,-2};
        }
    }

    public ArrayList<String> get_moves_agent_goal(Agent agent, SearchEngine searchEngine, MapFixedObjects mapFixedObjects){
        Optional<Box> next_box = mapFixedObjects.getNextBoxBy(agent.getColor());
        Box box_to_search;
        if (next_box.isPresent()){
            box_to_search=next_box.get();
        }else {
            box_to_search = null;
            System.err.println("#tried to get null box");
            System.exit(-1);
        }

        Stack<int[]> agent_path = searchEngine.runAstar(mapFixedObjects, agent.getColor(), agent.getCoordinates(), box_to_search.getCoordinates());
        int[] agent_goal = agent_path.pop();
        assert Arrays.equals(agent_goal, box_to_search.getCoordinates());
        int[] agent_end_path = agent_path.peek();
        ArrayList<String> agent_moves = this.getMoves(agent_path);

        Stack<int[]> box_path = searchEngine.runAstar(mapFixedObjects, box_to_search.getColor(), box_to_search.getCoordinates(), box_to_search.getGoalPosition());
        ArrayList<String> box_moves = this.getBoxMoves(agent, agent_end_path, box_path);

        box_moves.addAll(agent_moves);

        return box_moves;
    }
}