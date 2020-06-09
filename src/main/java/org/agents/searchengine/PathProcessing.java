package org.agents.searchengine;

import org.agents.Agent;
import org.agents.Box;
import org.agents.MapFixedObjects;
import org.agents.action.Direction;
import org.agents.markings.Coordinates;
import org.agents.planning.schedulling.SearchScheduled;
import org.agents.searchengine.normal.SearchEngineSANormal;

import java.util.*;
import java.util.stream.Stream;

public final class PathProcessing {
    public static final String MoveAction = "Move";
    public static final String PushAction = "Push";
    public static final String PullAction = "Pull";

    private int clock_time = 0;

    private ArrayList<String> getMAAgentMoves(ArrayDeque<int[]> path){
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

    public ArrayList<String[]> getMAAgentMoves(ArrayDeque<int[]> path, int[] index_agents){
        if(path.size()==0) return new ArrayList<>();

        ArrayList<String[]> agent_moves = new ArrayList<>();

        int[] next_cell = path.pop();
        String[] time_step_moves = new String[index_agents.length];
        while (!path.isEmpty()){
            int[] prev_cell = path.pop();

            Direction[] next_direction = Direction.getDirectionsFrom(prev_cell, next_cell, index_agents);
            for (int i = 0; i < next_direction.length; i++) {
                String dir_str = next_direction[i].toString();
                if (dir_str.equals("NoOp")){
                    time_step_moves[i] = dir_str;
                }else{
                    //time_step_moves[i] = MoveAction + "(" + next_direction[i].toString() + ")";
                    time_step_moves[i] = MoveAction + "(" + dir_str + ")";
                }
            }
            agent_moves.add(time_step_moves);
            time_step_moves = new String[index_agents.length];
            next_cell = prev_cell;
        }
        return agent_moves;
    }

    public synchronized ArrayList<String[]> getMAAgentBoxesMoves(SearchTaskResult searchTaskResult){
        ArrayDeque<int[]> path =  searchTaskResult.getPath();
        int[] index_agents =  searchTaskResult.getTotalGroup()[SearchScheduled.INDEX_OF_AGENTS];          ;
        HashMap<Integer,int[]> agents_idx_to_boxes_idx = searchTaskResult.getAgentstIdxsToBoxesIdxs();

        assert path != null;

        ArrayList<String[]> agent_moves = new ArrayList<>();

        int[] next_cell = path.pop();
        String[] time_step_moves;
        ArrayList<int[]> output_pairs = new ArrayList<>();
        HashMap<Integer,Set<Integer>> boxes_idx_moved = new HashMap<>();
        Set<Integer> agts_not_moved = new HashSet<>();
        while (!path.isEmpty()){
            int[] prev_cell = path.pop();
            //if prev_cell not goal next_cell agents_idx_to_boxes_idx[index_agents[i]]

            //  keep tracked if the box moved
            boolean box_moved = false;
            output_pairs.clear();
            boxes_idx_moved.clear();
            agts_not_moved.clear();
            for(Integer agents_idx : agents_idx_to_boxes_idx.keySet()){
                for (int bx_i = 0; bx_i < agents_idx_to_boxes_idx.get(agents_idx).length ; bx_i++) {
                    int bx_idx = agents_idx_to_boxes_idx.get(agents_idx)[bx_i];
                    if(Coordinates.areNeighbours( Coordinates.getRow(bx_idx, prev_cell), Coordinates.getCol(bx_idx, prev_cell), Coordinates.getRow(bx_idx, next_cell), Coordinates.getCol(bx_idx, next_cell) )){
                        box_moved = true;
                        if(Coordinates.areNeighbours( Coordinates.getRow(agents_idx, prev_cell), Coordinates.getCol(agents_idx, prev_cell), Coordinates.getRow(bx_idx, prev_cell), Coordinates.getCol(bx_idx, prev_cell) )){
                            if(Coordinates.areNeighbours( Coordinates.getRow(agents_idx, next_cell), Coordinates.getCol(agents_idx, next_cell), Coordinates.getRow(bx_idx, next_cell), Coordinates.getCol(bx_idx, next_cell) )){
                                //agents_idx moved from prev_cell along with the box
                                if(boxes_idx_moved.containsKey(bx_idx)){
                                    ( boxes_idx_moved.get(bx_idx)).add(agents_idx);
                                    //int[] pair = new int[]{agents_idx, bx_idx};
                                    //output_pairs.add(pair);//add the pair agent_idx box_idx to candidates  push and pulls
                                }else {
                                    Set<Integer> idxs = new HashSet<>();
                                    idxs.add(agents_idx);
                                    boxes_idx_moved.put(bx_idx, idxs);
                                }
                            }
                        }
                    }else{
                        agts_not_moved.add(agents_idx);
                    }
                }
            }
//       boolean neighbours_found = Coordinates.getNeighboursByIndexRanges(next_cell, index_agents, agents_idx_to_boxes_idx, output_pairs);
          //  boolean neighbours_found = Coordinates.getNeighboursByIndexRanges(prev_cell, index_agents, agents_idx_to_boxes_idx, output_pairs);

            time_step_moves = new String[index_agents.length];
            if(!box_moved){
                //prev_cell_neighbours = false;
                Direction[] next_direction = Direction.getDirectionsFrom(prev_cell, next_cell, index_agents);
                for (int indx = 0; indx < next_direction.length; indx++) {
                    time_step_moves[indx] = MoveAction + "(" + next_direction[indx].toString() + ")";
                }

            }else{
                Direction[] next_direction = Direction.getDirectionsFrom(prev_cell, next_cell, index_agents);
                for (int indx = 0; indx < next_direction.length; indx++) {
                    if( agts_not_moved.contains(index_agents[indx]) ){
                        time_step_moves[indx] = MoveAction + "(" + next_direction[indx].toString() + ")";
                    }
                }

                //make pulls  or pushes
                for (Integer bx_indx : boxes_idx_moved.keySet()){
                    Set<Integer> agt_indexes = boxes_idx_moved.get(bx_indx);
                    for (Integer agt_idx : agt_indexes){
                        String agent_dir = next_direction[agt_idx].toString();

                        if(Coordinates.getRow(agt_idx, prev_cell) == Coordinates.getRow(bx_indx, next_cell) &&
                                Coordinates.getCol(agt_idx, prev_cell) == Coordinates.getCol(bx_indx, next_cell)){
                            //make a pull action
                            Direction next_box_dir1 = Direction.getDirectionsFrom(Coordinates.getRow(bx_indx, next_cell), Coordinates.getCol(bx_indx, next_cell),
                                                                            Coordinates.getRow(bx_indx, prev_cell), Coordinates.getCol(bx_indx, prev_cell));

                            String str_move_pull = PullAction + "(" + agent_dir.toString() + "," + next_box_dir1.toString() + ")";
                            //boxes_idx_moved.remove(bx_indx); concurent modification

                            for (int i = 0; i < index_agents.length; i++) {
                                if (index_agents[i] == agt_idx)
                                    time_step_moves[i] = str_move_pull;
                            }
                        }else{
                            if(Coordinates.getRow(agt_idx, next_cell) == Coordinates.getRow(bx_indx, prev_cell) &&
                                    Coordinates.getCol(agt_idx, next_cell) == Coordinates.getCol(bx_indx, prev_cell)){
                            //make a push action
                                Direction next_box_dir2 = Direction.getDirectionsFrom(Coordinates.getRow(bx_indx, prev_cell), Coordinates.getCol(bx_indx, prev_cell),
                                                                                Coordinates.getRow(bx_indx, next_cell), Coordinates.getCol(bx_indx, next_cell));

                                String str_move_push = PushAction + "(" + agent_dir.toString() + "," + next_box_dir2.toString() + ")";
                                //boxes_idx_moved.remove(bx_indx);  concurent modification
                                for (int i = 0; i < index_agents.length; i++) {
                                    if (index_agents[i] == agt_idx)
                                        time_step_moves[i] = str_move_push;
                                }
                            }
                        }
                    }
                }
                    }
            agent_moves.add(time_step_moves);
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
        SearchTaskResult searchTaskResult = searchEngine.getPath();
        ArrayDeque<int[]> agent_path = searchTaskResult.getPath();
        int[] agent_goal = agent_path.pop();
        assert (Coordinates.getRow(agent_goal) == box_to_search.getRowPosition()) && (Coordinates.getCol(agent_goal) == box_to_search.getColumnPosition());
        int[] agent_end_path = agent_path.peek();
        ArrayList<String> agent_moves = this.getMAAgentMoves(agent_path);


        searchEngine.runAstar(box_to_search);
        SearchTaskResult searchTaskResult1 = searchEngine.getPath();
        ArrayDeque<int[]> box_path = searchTaskResult1.getPath();

        ArrayList<String> box_moves = this.getBoxMoves(agent, agent_end_path, box_path);

        box_moves.addAll(agent_moves);

        return box_moves;
    }

    public ArrayList<String> get_moves_agent_goal(Agent agent, SearchEngineSANormal searchEngineSANormal){
        Optional<Box> next_box = MapFixedObjects.getNextBoxBy(agent.getColor());
        Box box_to_search;
        if (next_box.isPresent()){
            box_to_search = next_box.get();
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
        ArrayList<String> agent_moves = this.getMAAgentMoves(agent_path);


        searchEngineSANormal.runAstar(box_to_search);
        ArrayDeque<int[]> box_path = searchEngineSANormal.getPath();
        ArrayList<String> box_moves = this.getBoxMoves(agent, agent_end_path, box_path);

        box_moves.addAll(agent_moves);

        return box_moves;
    }

    public ArrayList<String[]> outputPathsMA(ArrayDeque<int[]> agents_paths){
        int[] goal_cell = agents_paths.pop();
        int[] agent_end_path = agents_paths.peek();

        int[] multiple_agents = new int[goal_cell.length];
        for (int i = 0; i < goal_cell.length ; i++) {
            multiple_agents[i] = i;
        }

        ArrayList<String[]> agent_moves = this.getMAAgentMoves(agents_paths, multiple_agents);
        //boxes path to be added
        return agent_moves;
    }

//changes the time steps based on the previouse time steps of other paths changed
    public void resetTimeSteps(ArrayDeque<int[]> new_path_one) {
        if (new_path_one.size()==0) return;

        //int time_steps = new_path_one.size();
        this.clock_time += new_path_one.size();
        int time_steps = this.clock_time;
        int number_of_movable = (new_path_one.peek()).length/Coordinates.getLenght();

        for (int[] cell_pos: new_path_one){
            --time_steps;
            for (int coordinate = 0; coordinate < number_of_movable; coordinate++) {
                Coordinates.setTime(coordinate, cell_pos, time_steps);
            }
        }
    }

    //removes the goals coordinates of the agents becuase goals cell are box position cells
    //and box position cells are included in the path , this can be generalized for other goals
    public int[] getValidAgentsGoalCoordinates(SearchTaskResult searchTaskResult) {
        ArrayDeque<int[]> path_found = searchTaskResult.getPath();

        if(path_found.size()==0) return new int[0];

        int[] goal_coordinates = path_found.pop();

        int number_of_movables = Coordinates.getNumberOfCoordinates(goal_coordinates);
        ArrayDeque<int[]> coordinates_removed = new ArrayDeque<>();

        int[] __coordinates;

        int removedsize = 1;//size bigger than coordinates_removed.size()
        while (removedsize > coordinates_removed.size()) { //if coordinates chnged continue to scan for more
            removedsize = coordinates_removed.size();

            for (int index = 0; index < number_of_movables; index++) {
                __coordinates = path_found.pop();

                int row_pos = Coordinates.getRow(index, __coordinates);
                int col_pos = Coordinates.getCol(index, __coordinates);

                int row_goal = Coordinates.getRow(index, goal_coordinates);
                int col_goal = Coordinates.getCol(index, goal_coordinates);

                if (row_goal == row_pos || col_goal == col_pos) {
                    int[] __next_coordinates = path_found.peek();
                    assert __next_coordinates != null;
                    int next_row = Coordinates.getRow(index, __next_coordinates);
                    int next_col = Coordinates.getCol(index, __next_coordinates);

                    Coordinates.setRow(index, __coordinates, next_row);
                    Coordinates.setCol(index, __coordinates, next_col);

                    coordinates_removed.push(__coordinates);
                }
            }
        }

        while (!coordinates_removed.isEmpty()) path_found.add(coordinates_removed.pop());

        return  path_found.peek();
    }
}