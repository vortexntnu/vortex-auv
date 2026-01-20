#include "Audio.h"


TDT4102::Audio::Audio() {}

TDT4102::Audio::Audio(const std::filesystem::path pathToAudioFile) {
    src = pathToAudioFile;
}

void TDT4102::Audio::load() {
    if(!std::filesystem::exists(src)) {
        throw std::runtime_error("The audio file located at: " + src.string() + "\ncould not be found.");
    }

    if (Mix_OpenAudio(MIX_DEFAULT_FREQUENCY, MIX_DEFAULT_FORMAT, MIX_DEFAULT_CHANNELS, 2048) < 0) {
        throw std::runtime_error("Failed to open the audio device.\nError details: " + std::string(Mix_GetError()));
    }
    
    if (src.string().ends_with(".wav")) { // assume .wav is sound effect
        sfx = Mix_LoadWAV(src.string().c_str());
        if (sfx == NULL) {
            throw std::runtime_error("Failed to load sound effect. \nError details: " + std::string(Mix_GetError()));
        }
    } else if (src.string().ends_with(".mp3")) { // assume .mp3 is music
        mus = Mix_LoadMUS(src.string().c_str());
        if (mus == NULL) {
            throw std::runtime_error("Failed to load music. \nError details: " + std::string(Mix_GetError()));
        }
        isMusic = true;
    } else {
        throw std::runtime_error("Unrecognized audio format! Please only use .wav or .mp3 files");
    }

    ready = true;    
    
}

void TDT4102::Audio::play(int loops) {

    if (loops < 0) {
        throw std::runtime_error("Number of loops must be positive!");
    }
    
    if(!ready) {
        load();
    }
    if (isMusic) {
        Mix_PlayMusic(mus, loops == 0 ? loops : loops--);
    } else {
        Mix_PlayChannel(-1, sfx, loops == 0 ? loops-- : loops);
    }
    
    
}

TDT4102::Audio::~Audio() {
    if (sfx != nullptr) {
        Mix_FreeChunk(sfx);
        sfx = nullptr;
    }

    if (mus != nullptr) {
        Mix_FreeMusic(mus);
        mus = nullptr;
    }
    Mix_CloseAudio();
    Mix_Quit();
}

