package org.agents.planning.schedulling;

import org.agents.markings.Coordinates;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.atomic.AtomicInteger;

public class Synchronization {
    private static final int MINIMUM_CLOCK_TIME = 0;
    private static final int MAXIMUM_CLOCK_TIME = 1;

    private HashMap<Integer, ArrayList<Integer> > agents_clocks;
    private HashMap<Integer, ArrayList<Integer> > box_clocks;

    private final AtomicInteger clock_central_time;

    public Synchronization() {
        //Synchronization + in MovablesScheduling or
        this.clock_central_time = new AtomicInteger(0);
    }

    public void processTaskScheduled(TaskScheduled pathDescentralized){
        HashMap<Integer, ArrayDeque<int[]>> agents_paths = pathDescentralized.getAgentsToPaths();
        int[] clocks_agents = getMinMaxTime(agents_paths);
        int max_path_time1 = clocks_agents[MAXIMUM_CLOCK_TIME];

        HashMap<Integer, ArrayDeque<int[]>> boxes_paths = pathDescentralized.getBoxesToPaths();
        int[] clocks_boxes = getMinMaxTime(boxes_paths);
        int max_path_time2 = clocks_boxes[MAXIMUM_CLOCK_TIME];

        int max_time = Math.max(max_path_time1, max_path_time2);
        int clock_time = clock_central_time.get();

        if(max_time > clock_time)
            this.clock_central_time.set(max_time);
    }

    private int[] getMinMaxTime(HashMap<Integer, ArrayDeque<int[]>> paths){
        int max_path_time = 0;

        int min_path_time = -1;
        int first_time_step_temp = -1;
        int last_time_step_temp = -1;
        for (Integer key : paths.keySet()){
            ArrayDeque<int[]> path = paths.get(key);
            int[] first_location = path.peekFirst();
            int[] last_location = path.peekLast();
            first_time_step_temp = Coordinates.getTime(first_location);
            //assert min_path_time >= clock_central_time.get();

            last_time_step_temp = Coordinates.getTime(last_location);
            if(max_path_time < last_time_step_temp)
                max_path_time = last_time_step_temp;

            if(min_path_time > first_time_step_temp)
                min_path_time = first_time_step_temp;
        }

        return new int[]{min_path_time, max_path_time};
    }

    public void resetCentralTime(){
        this.clock_central_time.set(0);
    }

    public int increaseCentralTime(int  time_step){
        return this.clock_central_time.addAndGet(time_step);
    }

    public int decreaseCentralTime(int time_step){
        return this.clock_central_time.addAndGet(-time_step);
    }

    public int getCentralTime(){
        return this.clock_central_time.get();
    }
}
