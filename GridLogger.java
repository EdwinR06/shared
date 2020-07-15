package org.firstinspires.ftc.teamcode.shared;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

public class GridLogger {
    private static final String TIME = "Time";
    private Clock clock;
    private double elapsedTime;
    private LogWriter writer;
    private HashSet<String> categorySet;
    private HashMap<String, String> rowData;
    private boolean firstRow = true;
    private ArrayList<String> categories = new ArrayList<>();


    public GridLogger(LogWriter writer, Clock clock) {
        this.writer = writer;
        this.clock = clock;
        categorySet = new HashSet<>();
        rowData = new HashMap<>();
        clock.reset();
        categories.add(TIME);
    }

    /**
     * Add a value to the logger under the category.  Categories are lazily added to the logger
     * in the order encountered.
     *
     * @param category
     * @param value
     */
    public void add(String category, String value) {
        if (firstRow && !categorySet.contains(category)) {
            //Add the category to our list and set of categories
            categorySet.add(category);
            categories.add(category);
        }
        rowData.put(category, value);
    }

    public void add(String category, long value){
        add(category, Long.toString(value));
    }

    /**
     * Write a line of data to the log.  If this is the first call to writeLn, categories are
     * written first, followed by the line of data.  Once the data is written, the logger is reset
     * and calls to add() will add values to the next line of data.
     */
    public void writeLn() {

        //Write the title row if this is the first call to writeLn
        if (firstRow) {
            StringBuffer titleLine = new StringBuffer();
            for (int i = 0; i < categories.size(); i++) {
                titleLine.append(categories.get(i));
                if (i < categories.size() - 1) {
                    titleLine.append(",");
                }
            }
            writer.writeLine(titleLine.toString());
            firstRow = false;
        }

        //Update the time
        add(TIME, clock.getCurrentTime());
        //Write the data row
        StringBuffer dataRow = new StringBuffer();
        for (int i = 0; i < categories.size(); i++) {
            String category = categories.get(i);
            String value = rowData.get(category);
            if(value != null){
                dataRow.append(value);
            }
            //Avoid adding a comma after the last value
            if (i < categories.size() - 1) {
                dataRow.append(",");
            }
        }
        writer.writeLine(dataRow.toString());
        rowData.clear();
    }

    public void stop() {
    }
}


