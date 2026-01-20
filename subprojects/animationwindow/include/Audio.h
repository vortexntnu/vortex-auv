#pragma once

#include <string>
#include <filesystem>
#include <SDL_mixer.h>

namespace TDT4102 {
    struct Audio {
        // Allow the audio to be read out and played despite being private
        friend class AnimationWindow;

    public:
        explicit Audio();
        explicit Audio(std::filesystem::path pathToAudioFile);
        ~Audio();


    private:
        Mix_Chunk *sfx = nullptr;
        Mix_Music *mus = nullptr;
        bool ready = false; // flag to load audio before playing
        bool isMusic = false;

        std::filesystem::path src = "non existent file";

        void load();
        void play(int loops = 0);
    };
}
