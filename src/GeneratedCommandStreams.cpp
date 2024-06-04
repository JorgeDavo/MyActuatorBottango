#include "GeneratedCommandStreams.h"
#include <ArduinoJson.h>
#include <SD.h>

// SD card pins
#define SD_CS 13
#define SD_MOSI 15
#define SD_MISO 2
#define SD_SCK 14
namespace GeneratedCommandStreams
{
    CommandStream *setupStream = nullptr;
    CommandStream *stream_DefaultAnimation = nullptr;
    // Variables to store the animation data
    String SETUP_CHAR_STREAM;
    String STREAM_DEFAULTANIMATION_CHAR_STREAM;
    String STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP;

    bool playBegan = false;
    bool playOnStart = false; // Modified: set playOnStart to false
    bool loop = true;
    byte startingAnimation = 0;

    // called once per update loop, your logic probably goes here.
    void updatePlayStatus()
    {
        static unsigned long startTime = millis(); // Added: variable to store the start time

        // if haven't started any animations, or if should loop and nothing is playing (IE the last loop ended)
        if (!playBegan && millis() - startTime >= 2000) // Modified: check if 5 seconds have passed
        {
            BottangoCore::commandStreamProvider.startCommandStream(startingAnimation, loop);
            playBegan = true;
        }
    }

    void SDSetup()
    {

        // Set up the SD card pins
        SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

        // Initialize the SD card
        if (!SD.begin(SD_CS))
        {
            Serial.println("SD card initialization failed!");
            return;
        }
        Serial.println("SD card initialized.");

        // Open the JSON file
        File file = SD.open("/AnimationCommands.json");
        if (!file)
        {
            Serial.println("Failed to open file!");
            return;
        }

        // Read the file into a string
        String jsonData = "";
        while (file.available())
        {
            jsonData += (char)file.read();
        }

        // Parse the JSON data
        StaticJsonDocument<4096> doc; // Adjust the size to fit your JSON data
        DeserializationError error = deserializeJson(doc, jsonData);
        if (error)
        {
            Serial.println("Failed to parse JSON!");
            return;
        }

        // Access the animation data
        SETUP_CHAR_STREAM = doc[0]["Setup"]["Controller Setup Commands"].as<String>();
        STREAM_DEFAULTANIMATION_CHAR_STREAM = doc[0]["Animations"][0]["Animation Commands"].as<String>();
        STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP = doc[0]["Animations"][0]["Animation Loop Commands"].as<String>();

        // Close the file
        file.close();
        Serial.println("Animation Name: " + doc[0]["Animations"][0]["Animation Name"].as<String>());

        CommandStream setupStream = CommandStream(SETUP_CHAR_STREAM.c_str(), 0);
        CommandStream stream_DefaultAnimation = CommandStream(STREAM_DEFAULTANIMATION_CHAR_STREAM.c_str(), 10000, STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP.c_str(), 0);
    }

    CommandStream *getCommandStream(byte streamID)
    {
        switch (streamID)
        {
        case 0:
            return stream_DefaultAnimation;
        default:
            return NULL;
        }
    }

    CommandStream *getSetupCommandStream()
    {
        return setupStream;
    }
} // namespace GeneratedCommandStreams
