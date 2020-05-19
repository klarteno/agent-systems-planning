package datastructures;

import java.util.ArrayList;
import java.util.Arrays;

public class HungarianAlgorithmResizable {
    private final int[][] data_matrix;
    private final int[][] data_matrix_clone;
    private int data_max_column;
    private boolean is_rows_resizable = false;
    private boolean is_columns_resizable = false;
    private int rows_no;
    private int columns_no;

    public HungarianAlgorithmResizable(int[][] dataMatrix) {
        this.data_matrix = dataMatrix;
        this.data_matrix_clone = cloneAndPadMatrix(dataMatrix);
    }

    private int[][] cloneAndPadMatrix(int[][] dataMatrix){
        this.columns_no = dataMatrix[0].length;
        for (int[] row : dataMatrix) {
            if (columns_no < row.length) {
                columns_no = row.length;
                throw new IndexOutOfBoundsException("different number of columns for hungarian algorithm");
            }
        }

        this.rows_no = dataMatrix.length;

        int max_dimension;
        if(columns_no > rows_no){
            this.is_rows_resizable = true;
            max_dimension = columns_no;
        }else if(columns_no < rows_no){
            this.is_columns_resizable = true;
            max_dimension = rows_no;
        }else {
            max_dimension = columns_no;
        }
        //int max_dimension = Math.max(columns_no, rows_no);

        int[][] data_matrix_clone = new int[max_dimension][max_dimension];
        for (int row = 0; row < rows_no; row++) {
            /*for (int column = 0; column < dataMatrix[row].length; column++) {
                data_matrix_clone[row][column] = dataMatrix[row][column];
            }*/
            System.arraycopy(dataMatrix[row], 0, data_matrix_clone[row], 0, dataMatrix[row].length);
        }

        return data_matrix_clone;
    }

    //arrays of columns indices and row indices in the orignal array
    //to get the values index with this arraya in the inputes matrix
    public ArrayList<int[]> getMinimumCost(){
        //find optimal assignment
        //HungarianAlgorithm ha = new HungarianAlgorithm(this.data_matrix_clone);
        //int[][] assignment = ha.findOptimalAssignment();

        int[][] assignment = new HungarianAlgorithm(this.data_matrix_clone).findOptimalAssignment();

        ArrayList<int[]> assigns = new ArrayList<>();
        //removeExtraColsRows(assignment, assigns);
        if(this.is_columns_resizable)
            removeExtraCols(assignment, assigns, this.columns_no);
        if(this.is_rows_resizable)
            removeExtraRows(assignment, assigns, this.rows_no);
        else assigns.addAll(Arrays.asList(assignment));

        return assigns;
    }


    private static void removeExtraRows(int[][] assignment, ArrayList<int[]> assigns, int rows_no) {
        for (int[] pair : assignment) {
            int row_number = pair[1];
            //int  row_number  = assignment[assignment_no][1];
            if (row_number < rows_no) {
                assigns.add(pair);
            }
        }
    }

    private static void removeExtraCols(int[][] assignment, ArrayList<int[]> assigns, int columns_no) {
        for (int[] pair : assignment) {
            int column_number = pair[0];
            //int  row_number  = assignment[assignment_no][1];
            if (column_number < columns_no) {
                assigns.add(pair);
            }
        }
    }

    private static void removeNullsToNotUse(int[][] assignment) {
        int[] temp;
        int last_non_null = assignment.length-1;
        for (int assignment_no = 0; assignment_no < assignment.length; assignment_no++) {
            if (assignment[assignment_no] == null){
                for (int i = last_non_null; i >= 0 ; i--) {
                    if(assignment[i] != null){
                        temp = assignment[i];
                        assignment[i] = assignment[assignment_no];
                        assignment[assignment_no] = temp;
                        break;
                    }
                }

            }
        }
    }


}
