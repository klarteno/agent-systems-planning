package org.agents.searchengine;

import org.agents.markings.Coordinates;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Optional;

public class ODNodeStructure {
    private final HashMap<Integer, int[]> intermediate_came_from_standard;
    private final HashMap<Integer, int[]> standard_came_from_intermediate;
    private final HashMap<Integer, int[]> intermediate_came_from_intermediate;

    public ODNodeStructure() {
        intermediate_came_from_standard = new HashMap<>();
        standard_came_from_intermediate = new HashMap<>();
        intermediate_came_from_intermediate = new HashMap<>();

    }

    public boolean clear(){
        if(intermediate_came_from_standard.size()>0){
            intermediate_came_from_standard.clear();
            standard_came_from_intermediate.clear();
            intermediate_came_from_intermediate.clear();

            return true;
        }else {

            return false;
        }
    }




    public void updateIntermediateFromStandard(int[] state, int[] previouse_state) {
        intermediate_came_from_standard.put( Arrays.hashCode(state), previouse_state);
    }

    public int[] updateStandardFromIntermediate(int[] state, int[] current_state) {
        //how to prune
        standard_came_from_intermediate.put( Arrays.hashCode(state), current_state);
        int[] pos = state;
        int _pos_key = Arrays.hashCode(pos);
        int[] intermadiate_state = standard_came_from_intermediate.get(_pos_key);
        int intermadiate_state_key = Arrays.hashCode(intermadiate_state);
        int[] intermadiate_state2 = intermadiate_state;

        while(intermediate_came_from_intermediate.containsKey(intermadiate_state_key)){
            intermadiate_state2 = intermadiate_state;
            intermadiate_state_key =  Arrays.hashCode(intermediate_came_from_intermediate.get(intermadiate_state_key)) ;
            //intermediate_came_from_intermediate.remove(intermadiate_state2,intermadiate_state);
        }
        int[] prev_standard_state = intermediate_came_from_standard.get(intermadiate_state_key);
        //intermediate_came_from_standard.remove(intermadiate_state, prev_standard_state);
        return prev_standard_state;
    }

    public void updateIntermediateFromIntermediate(int[] state, int[] current_state) {
        intermediate_came_from_intermediate.put( Arrays.hashCode(state), current_state);
    }

    public Optional<int[]> getIntermediateNodeRoot(int[] _key_pos){
        int _intermadiate_state_key = Arrays.hashCode(_key_pos);
        int[] prev_standard_state__ = new int[0];
        Optional<int[]> prev_standard_state =  Optional.empty();

        if(intermediate_came_from_intermediate.containsKey(_intermadiate_state_key)){
            while(intermediate_came_from_intermediate.containsKey(_intermadiate_state_key)){
                _key_pos = intermediate_came_from_intermediate.get(_intermadiate_state_key);
                _intermadiate_state_key =  Arrays.hashCode(_key_pos) ;
            }
            prev_standard_state= Optional.ofNullable(intermediate_came_from_standard.get(_intermadiate_state_key));
            //prev_standard_state__ = intermediate_came_from_standard.get(_intermadiate_state_key);
        }
        if(intermediate_came_from_standard.containsKey(_intermadiate_state_key)){
            //prev_standard_state__ = intermediate_came_from_standard.get(_intermadiate_state_key);
            prev_standard_state = Optional.ofNullable(intermediate_came_from_standard.get(_intermadiate_state_key));
            //int prev_standard_state_key = Arrays.hashCode(prev_standard_state__);
        }
        return prev_standard_state;
    }


    public boolean isStandardNode(int[] pos_coordinates) {
        int prev_time = Coordinates.getTime(0, pos_coordinates);
        int next_time;
        int number_of_movables = pos_coordinates.length/Coordinates.getLenght();

        if(pos_coordinates.length > Coordinates.getLenght()){
            for (int coordinate = 0; coordinate < number_of_movables; coordinate = coordinate + 1) {
                next_time = Coordinates.getTime(coordinate, pos_coordinates);
                if (prev_time != next_time)
                    return false;
            }
        }
        return true;

    }
}