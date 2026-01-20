#pragma once

#include "KeyboardKey.h"
#include "SDL.h"

namespace TDT4102 {
    namespace internal {
        KeyboardKey convertSDLKeyToKeyboardKey(SDL_Keysym key);
    }
}