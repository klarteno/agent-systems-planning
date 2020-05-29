package org.agents.planning.conflicts;

import org.agents.markings.Coordinates;
import org.agents.planning.conflicts.dto.EdgeConflict;
import org.agents.planning.conflicts.dto.SimulationConflict;
import org.agents.planning.conflicts.dto.VertexConflict;

import java.util.ArrayList;

public final class IllegalPath {
    private final int[] start_group;
    private final ArrayList<SimulationConflict> paths_conflicts;
    private final int[] conflicting_group;

    private boolean is_checked;
    private ArrayList<int[][][]> start_group_paths_ordered;
    private final ArrayList<int[][][]> conflicting_paths_ordered;
    private int[] deadline_constraint;

    //start_group and pathsConflicts are ordered by the same marks ids
    public IllegalPath(int[] start_group, int[] conflictingGroup, ArrayList<SimulationConflict> pathsConflicts){
        this.start_group = start_group;
        this.paths_conflicts = pathsConflicts;
        this.conflicting_group = conflictingGroup;

        this.deadline_constraint = this.setDeadlineConstraint();
        this.is_checked = false;

        this.start_group_paths_ordered = new ArrayList<>();
        this.conflicting_paths_ordered = new ArrayList<>();
    }

    public void addPaths(int[][][] start_group_paths, int[][][] conflicting_paths){
        this.start_group_paths_ordered.add(start_group_paths);
        this.conflicting_paths_ordered.add(conflicting_paths);
    }

    public int[] getStartGroup(){
        return this.start_group;
    }

    public void setChecked(boolean isChecked){
        this.is_checked = isChecked;
    }

    public ArrayList<int[][][]> getPaths(){
        return this.conflicting_paths_ordered;
    }

    //for found paths take coordinates and find the conflicting coordinate in conflicting paths
    private int[] setDeadlineConstraint() {
        int[][] path;
        int time_deadline = -1;
        int row_deadline = -1;
        int column_deadline = -1;

        int[] coordinate = Coordinates.createCoordinates();

        for (SimulationConflict simulationConflict : paths_conflicts){
            int[] coordinate_max = simulationConflict.getMaxTimeDeadline();
            int time_step = Coordinates.getTime(coordinate_max);

            if(time_deadline < time_step){
                time_deadline = time_step;
                coordinate = coordinate_max;
            }
        }

        return coordinate;
    }

    private int[]  getLatestTimeForConflictingPath() {
        int[][] path;
        int time_deadline = -1;
        int row_deadline = -1;
        int column_deadline = -1;

        for (int[][][] conflicting_paths : conflicting_paths_ordered) {
            for (int[][] conflicting_path : conflicting_paths) {
                path = conflicting_path;
                for (int row = 0; row < path.length; row++) {
                    for (int column = 0; column < path[row].length; column++) {
                        if (path[row][column] > time_deadline) {
                            time_deadline = path[row][column];
                            row_deadline = row;
                            column_deadline = column;
                        }
                    }
                }
            }
        }

        return Coordinates.createCoordinates(time_deadline, row_deadline, column_deadline);
    }

    public int[] getDeadlineConstraint(){
        return this.deadline_constraint;
    }
}
