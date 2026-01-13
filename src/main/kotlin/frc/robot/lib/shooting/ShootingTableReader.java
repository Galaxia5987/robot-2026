package frc.robot.lib.shooting;

import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap;
import java.io.FileReader;
import java.io.IOException;

public class ShootingTableReader {

    public static InterpolatingDoubleMap parse(String file) {
        InterpolatingDoubleMap map = new InterpolatingDoubleMap(100);
        try (CSVReader reader = new CSVReader(new FileReader(file))) {
            String[] values;
            reader.readNext();
            while ((values = reader.readNext()) != null) {
                map.put(Double.parseDouble(values[0]), Double.parseDouble(values[1]));
            }
        } catch (CsvValidationException | IOException e) {
            throw new RuntimeException(e);
        }
        return map;
    }
}
