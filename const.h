#define WINDOW_TITLE                "ZX-Emulator-0"
#define Z80_FREQ                    3579545
//#define Z80_FREQ                    3546900
#define SCREEN_WIDTH                320
#define SCREEN_HEIGHT               240
#define AUDIO_RATE                  48000
#define CONFIG_FILE                 "emulator.cfg"
#define WINDOW_WIDTH                SCREEN_WIDTH * 3
#define WINDOW_HEIGHT               SCREEN_HEIGHT * 3

enum HW {
    SPECTRUM_128, SPECTRUM_48, PENTAGON_128
};
