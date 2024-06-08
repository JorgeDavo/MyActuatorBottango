#include "GeneratedCommandStreams.h"

namespace GeneratedCommandStreams
{

    //// !!! Insert/edit your animation triggering and playback logic here !!! ////
    /* Some helpful methods:

        BottangoCore::commandStreamProvider.streamIsInProgress()                                        //returns if any animation is currently playing
        BottangoCore::commandStreamProvider.startCommandStream(byte animationID, bool looping)          //starts playing an animation (and stops the current if any)
        BottangoCore::commandStreamProvider.stop()                                                      //stops the current playing animation, if any
    */

    /* Animations key:
        0 - Default Animation
    */

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
#ifdef USE_COMMAND_STREAM
            BottangoCore::commandStreamProvider.startCommandStream(startingAnimation, loop);
            if(BottangoCore::getAutoHomingStatus() == false){
                playBegan = true;
            }
#endif
            // playBegan = true;
        }
    }

    //// ----------------------------------------- /////

    //// GENERATED CODE. You probably don't need to change any of the below ////
    const char SETUP_CHAR_STREAM[] PROGMEM = "rMTR,1,1,3700,20000,1\nrMTR,2,1,3600,20000,533\nrMTR,3,1,10000,32767,2418\nrMTR,4,1,10000,32767,1480\nrMTR,5,1,10000,32767,1480\nrMTR,6,1,10000,32767,1480\nrMTR,7,1,10000,32767,1480\n";
    const char STREAM_DEFAULTANIMATION_CHAR_STREAM[] PROGMEM = "sC,1,0,1233,0,308,0,8192,-308,117\nsC,7,0,1233,1212,308,0,8192,-308,117\nsC,3,0,933,1980,233,0,4563,-233,88\nsC,2,0,1133,1212,283,0,3700,-283,107\nsC,6,0,767,1212,192,0,8192,-192,73\nsC,5,0,1233,1212,308,0,8192,-308,117\nsC,4,0,933,1212,233,0,8192,-233,88\nsC,6,767,1533,8192,383,-145,1415,-383,-223\nsC,3,933,634,4563,158,-60,1980,-158,-92\nsC,4,933,1500,8192,375,-142,1415,-375,-218\nsC,2,1133,834,3700,402,-152,1415,-524,-305\nsC,1,1233,734,8192,183,-70,1415,-183,-107\nsC,5,1233,734,8192,183,-70,1415,-183,-107\nsC,7,1233,734,8192,183,-70,1415,-183,-107\nsC,3,1567,2033,1980,508,295,0,-508,158\nsC,7,1967,1400,1415,350,203,247,-350,109\nsC,2,1967,1733,1415,730,424,8192,-433,134\nsC,1,1967,1400,1415,350,203,247,-350,109\nsC,5,1967,1833,1415,458,266,247,-458,142\nsC,6,2300,767,1415,192,111,247,-192,59\nsC,4,2433,600,1415,150,87,247,-150,47\nsC,4,3033,1367,247,342,-106,0,-342,-183\nsC,6,3067,1500,247,375,-116,0,-375,-201\nsC,1,3367,900,247,225,-70,0,-223,-285\nsC,7,3367,900,247,225,-70,0,-225,-121\nsC,3,3600,500,0,125,-39,1980,-125,-67\nsC,2,3700,567,8192,142,-44,0,-308,-165\nsC,5,3800,233,247,58,-18,0,-58,-31\nsC,5,4033,567,0,142,76,8192,-142,12\nsC,3,4100,800,1980,200,107,4563,-200,17\nsC,2,4267,2366,0,805,432,5325,-592,50\nsC,7,4267,633,0,158,85,8192,-158,13\nsC,1,4267,533,0,350,448,8192,-133,11\nsC,4,4400,1000,0,250,134,8192,-250,21\nsC,6,4567,633,0,158,85,8192,-158,13\nsC,5,4600,1900,8192,475,-40,0,-475,804\nsC,1,4800,1400,8192,350,-29,0,-350,592\nsC,3,4900,1500,4563,375,-31,1980,-375,635\nsC,7,4900,1767,8192,442,-37,0,-442,747\nsC,6,5200,733,8192,183,-15,0,-183,310\nsC,4,5400,567,8192,142,-12,0,-142,240\nsC,6,5933,1100,0,275,-465,5477,-275,-75\nsC,4,5967,933,0,233,-395,5477,-233,-64\nsC,1,6200,1033,0,279,-472,5477,-394,-107\nsC,3,6400,833,1980,208,-353,8192,-208,-57\nsC,5,6500,1100,0,275,-465,5477,-275,-75\nsC,2,6633,1300,5325,325,-27,450,-894,1513\nsC,7,6667,1100,0,275,-465,5477,-275,-75\nsC,4,6900,1033,5477,258,70,0,-258,532\nsC,6,7033,1934,5477,483,132,0,-483,996\nsC,1,7233,1367,5477,436,119,1336,-445,-1332\nsC,3,7233,1134,8192,283,77,1980,-283,584\nsC,5,7600,1033,5477,258,70,0,-258,532\nsC,7,7767,766,5477,192,52,0,-192,395\nsC,2,7933,2067,450,527,-893,1212,-517,0\nsC,4,7933,2067,0,517,-1064,1212,-517,0\nsC,3,8367,1633,1980,408,-841,1980,-408,0\nsC,7,8533,1467,0,367,-755,1212,-367,0\nsC,1,8600,1400,1336,546,1635,0,-357,0\nsC,5,8633,1367,0,342,-704,1212,-342,0\nsC,6,8967,1033,0,258,-532,1212,-258,0\n";
    const char STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP[] PROGMEM = "sC,1,10000,0,0,0,0,0,0,0\n";

    CommandStream setupStream = CommandStream(SETUP_CHAR_STREAM, 0);
    CommandStream stream_DefaultAnimation = CommandStream(STREAM_DEFAULTANIMATION_CHAR_STREAM, 10000, STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP, 0);

    CommandStream *getCommandStream(byte streamID)
    {
        switch (streamID)
        {
        case 0:
            return &stream_DefaultAnimation;
        default:
            return NULL;
        }
    }

    CommandStream *getSetupCommandStream()
    {
        return &setupStream;
    }
} // namespace GeneratedCommandStreams
