
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include "model.h"

// Initialize the infrared light sensor
Adafruit_TSL2591 lightSensor = Adafruit_TSL2591(2591);

// Sensor and communication pins
const int UV_SENSOR_PIN = 34;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// Sampling and filtering configuration
const float FILTER_ALPHA = 0.2f;
const unsigned long SAMPLE_PERIOD = 100;

// Store the filtered sensor measurements
float filteredUV = -1.0f;
float filteredIR = -1.0f;

// Keep track of sampling time and sample number
unsigned long previousSampleTime = 0;
unsigned long sampleCount = 0;


// Classify the sensor readings using the generated lookup table.
// Returns -1 if the prediction does not meet the rejection threshold.
int classifySensors(float uvValue, float irValue,
                    int &gridRow, int &gridCol, float &confidence) {

  // Apply the same normalization used during model training.
  float scaledUV =
      (uvValue - SCALE_BOUNDS[0]) /
      (SCALE_BOUNDS[1] - SCALE_BOUNDS[0]);

  float scaledIR =
      (log1pf(irValue) - SCALE_BOUNDS[2]) /
      (SCALE_BOUNDS[3] - SCALE_BOUNDS[2]);

  // Convert normalized values into lookup-table coordinates.
  gridCol = constrain(
      (int)(scaledUV * GRID_N), 0, GRID_N - 1
  );

  gridRow = constrain(
      (int)(scaledIR * GRID_N), 0, GRID_N - 1
  );

  // Search for the class with the highest stored value.
  int predictedClass = -1;
  confidence = -1.0f;

  for (int classIndex = 0; classIndex < NUM_CLASSES; classIndex++) {

    float currentValue =
        pgm_read_byte(
            &CLASS_TABLE[classIndex][gridRow][gridCol]
        ) / 255.0f;

    if (currentValue > confidence) {
      confidence = currentValue;
      predictedClass = classIndex;
    }
  }

  // Reject predictions below the configured threshold.
  if (confidence < REJECT_TAU) {
    return -1;
  }

  return predictedClass;
}


void setup() {

  Serial.begin(115200);

  // Configure the ESP32 analog-to-digital converter.
  analogReadResolution(12);

  // Start I2C communication with the infrared sensor.
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  if (!lightSensor.begin()) {
    Serial.println("# ERROR: TSL2591 not found.");

    while (true) {
      delay(1000);
    }
  }

  // These settings must match the data-collection sketch.
  lightSensor.setGain(TSL2591_GAIN_MED);
  lightSensor.setTiming(TSL2591_INTEGRATIONTIME_100MS);

  // Output column names for CSV logging.
 Serial.println("idx,rawUV,rawIR,uv,ir,row,col,class,confidence,latency_us");
}


void loop() {

  // Wait until the next sampling interval.
  unsigned long currentTime = millis();

  if (currentTime - previousSampleTime < SAMPLE_PERIOD) {
    return;
  }

  previousSampleTime = currentTime;

  // Read the ultraviolet sensor.
  int uvReading = analogRead(UV_SENSOR_PIN);

  // Read the infrared sensor.
  uint32_t lightData = lightSensor.getFullLuminosity();
  uint16_t irReading = lightData >> 16;

  // Initialize the filters with the first sensor readings.
  if (filteredUV < 0.0f) {

    filteredUV = uvReading;
    filteredIR = irReading;

  } else {

    // Apply exponential moving average filtering.
    filteredUV =
        FILTER_ALPHA * uvReading +
        (1.0f - FILTER_ALPHA) * filteredUV;

    filteredIR =
        FILTER_ALPHA * irReading +
        (1.0f - FILTER_ALPHA) * filteredIR;
  }

  // Variables used to store classification results.
  int detectedRow;
  int detectedCol;
  float predictionValue;

  // Measure classification execution time only.
  unsigned long startMicros = micros();

  int detectedClass = classifySensors(
      filteredUV,
      filteredIR,
      detectedRow,
      detectedCol,
      predictionValue
  );

  unsigned long classificationTime = micros() - startMicros;

  // Print the classification results as a CSV row.
  // Output raw and filtered sensor readings with classification results.
  Serial.print(sampleCount++);
  Serial.print(",");
  
  // Raw sensor readings before filtering
  Serial.print(uvReading);
  Serial.print(",");
  Serial.print(irReading);
  Serial.print(",");
  
  // Filtered sensor readings
  Serial.print(filteredUV, 2);
  Serial.print(",");
  Serial.print(filteredIR, 2);
  Serial.print(",");
  
  // Lookup-table coordinates
  Serial.print(detectedRow);
  Serial.print(",");
  Serial.print(detectedCol);
  Serial.print(",");
  
  // Predicted class
  if (detectedClass < 0) {
    Serial.print("UNKNOWN");
  } else {
    Serial.print(CLASS_NAMES[detectedClass]);
  }
  
  Serial.print(",");
  
  // Classification confidence score
  Serial.print(predictionValue, 4);
  Serial.print(",");
  
  // Classification execution time in microseconds
  Serial.println(classificationTime);
}
