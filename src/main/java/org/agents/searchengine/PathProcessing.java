package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.action.CellLocation;
import org.agents.action.Direction;
import org.agents.markings.Coordinates;
import org.agents.searchengine.normal.SearchEngineSANormal;

import java.util.*;
import java.util.stream.Stream;

public final class PathProcessing {
    public static final String MoveAction = "Move";
    public static final String PushAction = "Push";
    public static final String PullAction = "Pull";


    private ArrayList<String> getMoves(ArrayDeque<int[]> path){
        assert path != null;
        ArrayList<String> agent_moves = new ArrayList<>();
        int[] next_cell = path.pop();
        while (!path.isEmpty()){
            int[] prev_cell = path.pop();
            Direction next_direction = Direction.getDirectionsFrom(prev_cell, next_cell);
            agent_moves.add(MoveAction + "(" + next_direction.toString() + ")" );
            next_cell = prev_cell;
        }
        return agent_moves;
    }

    private ArrayList<String> getMoves(ArrayDeque<int[]> path, boolean is_multiple_agents ){
        assert path != null;
        ArrayList<String> agent_moves = new ArrayList<>();
        int[] next_cell = path.pop();
        String time_step_moves = "";
        while (!path.isEmpty()){
            int[] prev_cell = path.pop();
            Direction[] next_direction = Direction.getDirectionsFrom(prev_cell, next_cell, is_multiple_agents);
            for (int i = 0; i < next_direction.length; i++) {
                time_step_moves += MoveAction + "(" + next_direction[i].toString() + ")";
            }
            agent_moves.add(time_step_moves);
            time_step_moves = "";
            next_cell = prev_cell;
        }
        return agent_moves;
    }



    private ArrayList<String> getBoxMoves(Agent agent, int[] agent_cell, ArrayDeque<int[]> box_path){
        assert agent_cell != null;

        ArrayList<String> agent_moves = new ArrayList<>();

        Iterator<int[]> iter1 = box_path.iterator();
        int[] prev_cell1 = iter1.next();
        int[] next_cell1 = iter1.next();

        int[] prev_cell;
        int[] next_cell = box_path.pop();
      //  while(iter.hasNext()){
        while(!box_path.isEmpty()){
            prev_cell = box_path.pop();
            Direction position = Direction.getDirectionsFrom(agent_cell, prev_cell);
            if(Arrays.equals(next_cell, agent_cell)){
                ArrayDeque<int[]> cells = MapFixedObjects.getNeighbours(agent_cell, agent.getColor());
                Stream<int[]> next_neighbours = cells.stream().filter(box_path::contains);//to optimize and abstract
                int[] cell = this.selectCellForAgent(next_neighbours);
                Direction agent_dir = Direction.getDirectionsFrom(agent_cell, cell);
                agent_moves.add(PullAction + "(" + agent_dir.toString() + "," + position.toString() +")" );
                agent_dir.getNextCellFrom(agent_cell);
            }else {
                Direction next_box_dir = Direction.getDirectionsFrom(prev_cell, next_cell);
                agent_moves.add(PushAction + "(" + position.toString() + "," + next_box_dir.toString() +")" );

                agent_cell = next_cell;
            }
            next_cell = prev_cell;
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

    //this will not work without searchEngineSANormal
    public ArrayList<String> get_moves_agent_goal(Agent agent, SearchEngineSA searchEngine){
        Optional<Box> next_box = MapFixedObjects.getNextBoxBy(agent.getColor());
        Box box_to_search;
        if (next_box.isPresent()){
            box_to_search=next_box.get();
        }else {
            box_to_search = null;
            System.out.println("#tried to get null box");
            System.exit(-1);
        }

        agent.setGoalPosition(box_to_search.getRowPosition(), box_to_search.getColumnPosition());
        searchEngine.runAstar(agent);
        ArrayDeque<int[]> agent_path = searchEngine.getPath();
        int[] agent_goal = agent_path.pop();
        assert (Coordinates.getRow(agent_goal) == box_to_search.getRowPosition()) && (Coordinates.getCol(agent_goal) == box_to_search.getColumnPosition());
        int[] agent_end_path = agent_path.peek();
        ArrayList<String> agent_moves = this.getMoves(agent_path);


        searchEngine.runAstar(box_to_search);
        ArrayDeque<int[]> box_path = searchEngine.getPath();
        ArrayList<String> box_moves = this.getBoxMoves(agent, agent_end_path, box_path);

        box_moves.addAll(agent_moves);

        return box_moves;
    }


    public ArrayList<String> get_moves_agent_goal(Agent agent, SearchEngineSANormal searchEngineSANormal){
        Optional<Box> next_box = MapFixedObjects.getNextBoxBy(agent.getColor());
        Box box_to_search;
        if (next_box.isPresent()){
            box_to_search=next_box.get();
        }else {
            box_to_search = null;
            System.out.println("#tried to get null box");
            System.exit(-1);
        }

        agent.setGoalPosition(box_to_search.getRowPosition(), box_to_search.getColumnPosition());
        searchEngineSANormal.runAstar(agent);
        ArrayDeque<int[]> agent_path = searchEngineSANormal.getPath();
        int[] agent_goal = agent_path.pop();
        assert (Coordinates.getRow(agent_goal) == box_to_search.getRowPosition()) && (Coordinates.getCol(agent_goal) == box_to_search.getColumnPosition());
        int[] agent_end_path = agent_path.peek();
        ArrayList<String> agent_moves = this.getMoves(agent_path);


        searchEngineSANormal.runAstar(box_to_search);
        ArrayDeque<int[]> box_path = searchEngineSANormal.getPath();
        ArrayList<String> box_moves = this.getBoxMoves(agent, agent_end_path, box_path);

        box_moves.addAll(agent_moves);

        return box_moves;
    }



    public ArrayList<String> outputPathsMA(ArrayDeque<int[]> agents_paths){
        int[] goal_cell = agents_paths.pop();
        int[] agent_end_path = agents_paths.peek();
        boolean multiple_agents = true;
        ArrayList<String> agent_moves = this.getMoves(agents_paths, multiple_agents);

        //boxes path to be added
        return agent_moves;
    }

    public void resetTimeSteps(ArrayDeque<int[]> new_path_one) {
            int time_steps = new_path_one.size();
            int number_of_movable = (new_path_one.peek()).length/Coordinates.getLenght();

            for (int[] cell_pos: new_path_one){
                --time_steps;
                for (int coordinate = 0; coordinate < number_of_movable; coordinate++) {
                    Coordinates.setTime(coordinate, cell_pos, time_steps);
                }

            }
    }
}