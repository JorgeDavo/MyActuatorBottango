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
    bool playOnStart = false;
    bool loop = true;
    byte startingAnimation = 0;

    // called once per update loop, your logic probably goes here.
    void updatePlayStatus()
    {
#ifdef USE_COMMAND_STREAM
        // if haven't started any animations, or if should loop and nothing is playing (IE the last loop ended)
        if ((!playBegan || (loop && !BottangoCore::commandStreamProvider.streamIsInProgress())))
        {

            BottangoCore::commandStreamProvider.startCommandStream(startingAnimation, loop);
            if (BottangoCore::getAutoHomingStatus() == false)
            {
                playBegan = true;
            }
            // playBegan = true;
        }
#endif
    }

    //// ----------------------------------------- /////

    //// GENERATED CODE. You probably don't need to change any of the below ////
    const char SETUP_CHAR_STREAM[] PROGMEM = "rMTR,1,1,4000,3000,1\nrMTR,2,1,3156,3000,1\nrMTR,3,0,4144,3000,0\nrMTR,4,1,3197,3000,1\nrMTR,5,1,3197,3000,1\nrSVPin,32,900,1166,30000,900\n";
    const char STREAM_DEFAULTANIMATION_CHAR_STREAM[] PROGMEM = "sC,1,0,1633,0,408,0,5985,-408,0\nsC,2,0,2067,0,517,0,2710,-517,0\nsC,4,0,1933,0,483,0,3700,-483,0\nsC,5,0,4733,0,1183,0,3433,-1183,0\nsC,32,0,2900,0,725,0,3662,-725,0\nsC,3,0,1933,0,483,0,3700,-483,0\nsC,1,1633,2600,5985,663,0,44,-988,835\nsC,4,1933,1634,3700,408,0,0,-408,0\nsC,3,1933,1634,3700,408,0,0,-408,0\nsC,2,2067,2033,2710,508,0,158,-508,0\nsC,32,2900,2800,3662,700,0,3939,-700,0\nsC,3,3567,2966,0,742,0,8192,-742,0\nsC,4,3567,2966,0,742,0,8192,-742,0\nsC,2,4100,2267,158,567,0,8156,-567,0\nsC,1,4233,3134,44,1112,-940,8192,-860,-367\nsC,5,4733,3000,3433,750,0,2088,-750,0\nsC,32,5700,10000,3939,2500,0,3939,-2500,0\nsC,2,6367,2000,8156,500,0,3738,-500,0\nsC,3,6533,2067,8192,517,0,3890,-517,0\nsC,4,6533,2067,8192,517,0,3890,-517,0\nsC,1,7367,3000,8192,1069,456,1120,-927,0\nsC,5,7733,3400,2088,850,0,3022,-850,0\nsC,2,8367,1833,3738,458,0,903,-458,0\nsC,3,8600,2467,3890,617,0,6658,-617,0\nsC,4,8600,2467,3890,617,0,6658,-617,0\nsC,2,10200,4067,903,1017,0,8192,-1017,0\nsC,1,10367,3000,1120,702,0,7075,-1138,-1326\nsC,4,11067,3466,6658,867,0,4406,-867,0\nsC,3,11067,3466,6658,867,0,4406,-867,0\nsC,5,11133,3467,3022,867,0,8192,-867,0\nsC,1,13367,1633,7075,703,819,7476,-244,75\nsC,2,14267,1500,8192,375,0,3105,-375,0\nsC,4,14533,6467,4406,1617,0,1587,-1617,0\nsC,3,14533,6467,4406,1617,0,1587,-1617,0\nsC,5,14600,6133,8192,1533,0,5324,-1533,0\nsC,1,15000,1900,7476,797,-244,1070,-919,340\nsC,32,15700,2267,3939,567,0,7809,-567,0\nsC,2,15767,3100,3105,775,0,7142,-775,0\nsC,1,16900,4267,1070,3525,-1306,8192,-2673,-392\nsC,32,17967,3366,7809,842,0,5541,-842,0\nsC,2,18867,2766,7142,692,0,2721,-692,0\nsC,5,20733,6167,5324,1542,0,8192,-1542,0\nsC,3,21000,5800,1587,1450,0,1854,-1450,0\nsC,4,21000,5800,1587,1450,0,1854,-1450,0\nsC,1,21167,4100,8192,973,143,3806,-1811,-449\nsC,32,21333,4567,5541,1142,0,7192,-1142,0\nsC,2,21633,6967,2721,1742,0,7092,-1742,0\nsC,1,25267,5733,3806,1672,414,319,-1433,0\nsC,32,25900,3700,7192,925,0,4790,-925,0\nsC,4,26800,7600,1854,1900,0,6625,-1900,0\nsC,3,26800,7600,1854,1900,0,6625,-1900,0\nsC,5,26900,7533,8192,1883,0,3472,-1883,0\nsC,2,28600,3633,7092,908,0,1470,-908,0\nsC,32,29600,5967,4790,1492,0,7476,-1492,0\nsC,1,31000,3200,319,800,0,8192,-800,0\nsC,2,32233,4700,1470,1175,0,6825,-1175,0\nsC,1,34200,1200,8192,300,0,670,-300,0\nsC,3,34400,4667,6625,1167,0,3923,-1167,0\nsC,4,34400,4667,6625,1167,0,3923,-1167,0\nsC,5,34433,4667,3472,1167,0,7709,-1167,0\nsC,1,35400,1133,670,283,0,7642,-283,0\nsC,32,35567,5733,7476,1433,0,5524,-1433,0\nsC,1,36533,1200,7642,300,0,770,-300,0\nsC,2,36933,7267,6825,1817,0,0,-1817,0\nsC,1,37733,834,770,208,0,4239,-207,-156\nsC,1,38567,4500,4239,1043,782,4340,-589,0\nsC,3,39067,9466,3923,2367,0,7792,-2367,0\nsC,4,39067,9466,3923,2367,0,7792,-2367,0\nsC,5,39100,3033,7709,755,0,0,-1131,0\nsC,32,41300,4700,5524,1175,0,8192,-1175,0\nsC,5,42133,2067,0,699,0,8192,-672,-961\nsC,1,43067,3600,4340,900,0,7225,-900,0\nsC,5,44200,3967,8192,1516,2168,4923,-1052,0\nsC,2,44200,2767,0,692,0,6214,-692,0\nsC,32,46000,1033,8192,258,0,4990,-258,0\nsC,1,46667,2333,7225,583,0,2488,-1189,-550\nsC,2,46967,3566,6214,892,0,425,-892,0\nsC,32,47033,4100,4990,1025,0,6758,-1025,0\nsC,5,48167,7033,4923,1758,0,3243,-1758,0\nsC,4,48533,7367,7792,1842,0,219,-1842,0\nsC,3,48533,7367,7792,1842,0,219,-1842,0\nsC,1,49000,3300,2488,824,381,436,-825,0\nsC,2,50533,2600,425,650,0,6366,-650,0\nsC,32,51133,4234,6758,1058,0,4406,-1058,0\nsC,1,52300,2367,436,592,0,7409,-200,-524\nsC,2,53133,2567,6366,642,0,958,-642,0\nsC,1,54667,5333,7409,1272,3334,0,-1333,0\nsC,5,55200,4800,3243,1200,0,0,-1200,0\nsC,32,55367,2266,4406,567,0,6658,-567,0\nsC,2,55700,1967,958,492,0,6023,-492,0\nsC,4,55900,4100,219,1025,0,0,-1025,0\nsC,3,55900,4100,219,1025,0,0,-1025,0\nsC,32,57633,2367,6658,592,0,3322,-592,0\nsC,2,57667,2333,6023,583,0,0,-583,0\n";
    const char STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP[] PROGMEM = "sC,5,60000,0,0,0,0,0,0,0\nsC,32,60000,14,3322,7,0,0,-7,0\n";

    CommandStream setupStream = CommandStream(SETUP_CHAR_STREAM, 0);
    CommandStream stream_DefaultAnimation = CommandStream(STREAM_DEFAULTANIMATION_CHAR_STREAM, 60000, STREAM_DEFAULTANIMATION_CHAR_STREAM_LOOP, 14);    

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
