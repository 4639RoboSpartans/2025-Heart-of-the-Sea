package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class TunableTable {
    protected final String name;
    protected final String[] columnNames;
    protected final double[][] data;
    private final int numRows;
    private final int numCols;
    private final List<TableSourceListener> listeners = new ArrayList<>();
    private int currRow = 0;

    public interface TableSourceListener {
        void onTableChange(int row, int col, double value);
    }

    public TunableTable(String name, int numRows, int numCols, String[] columnNames, double[][] data) {

        this.name = name;
        this.numRows = numRows;
        this.numCols = numCols;
        this.columnNames = columnNames;
        this.data = data;

        // Add full display
        SmartDashboard.putData(name, builder -> {
            builder.setSmartDashboardType("Network Table Tree");
            for (int row = 0; row < numRows(); row++) {
                int finalRow = row;
                builder.addStringProperty("Row " + (row + 1), () -> {
                    StringBuilder s = new StringBuilder();
                    for (int col = 0; col < numColumns(); col++) {
                        s.append(getColumnName(col));
                        s.append("=");
                        s.append(getCellAsDouble(finalRow, col));
                        s.append(", ");
                    }
                    return s.toString();
                }, null);
            }
        });

        // Add row select
        SmartDashboard.putData("Row select for " + name, new SendableChooser<Integer>() {
            {
                setDefaultOption("Row 1", 1);
                for (int row = 1; row < numRows(); row++) {
                    addOption("Row " + (row + 1), row);
                }
                onChange(x -> updateCurrentRow(x));
            }
        });

        // Add column writers
        SmartDashboard.putData("Editor for " + name, builder -> {
            for (int column = 0; column < numColumns(); column++) {
                int finalColumn = column;
                builder.addDoubleProperty(
                    getColumnName(column),
                    () -> getCellAsDouble(currRow, finalColumn),
                    x -> {
                        data[currRow][finalColumn] = x;
                        for (TableSourceListener listener : listeners) {
                            listener.onTableChange(currRow, finalColumn, x);
                        }
                    }
                );
            }
        });
    }

    public int numRows() {
        return numRows;
    }

    public int numColumns() {
        return numCols;
    }

    public String getColumnName(int col) {
        return columnNames[col];
    }

    public double getCellAsDouble(int row, int col) {
        return data[row][col];
    }

    private void updateCurrentRow(int x) {
        currRow = x;
    }

    public void addListener(TableSourceListener listener) {
        listeners.add(listener);
    }
}
