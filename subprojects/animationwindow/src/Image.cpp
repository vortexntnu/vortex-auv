#include "Image.h"
#include <SDL_image.h>

TDT4102::Image::Image() {}

TDT4102::Image::Image(const std::filesystem::path pathToImageFile) {
    src = pathToImageFile;
}

void TDT4102::Image::load(SDL_Renderer* renderer) {
    if(!std::filesystem::exists(src)) {
        throw std::runtime_error("The image file located at: " + src.string() + "\ncould not be found.");
    }
    texture = IMG_LoadTexture(renderer, src.string().c_str());

    if(width == 0 && height == 0) {
        SDL_QueryTexture(texture, nullptr, nullptr, &width, &height);
    }
}

void TDT4102::Image::draw(SDL_Renderer *renderer, TDT4102::Point location, int imageWidth, int imageHeight) {
    // We need a reference to SDL_Renderer to perform the loading operation
    // This requires loading it once it gets used
    if(texture == nullptr) {
        load(renderer);
    }

    SDL_Rect imageBounds {location.x, location.y, width, height};

    if(imageWidth > 0 && imageHeight > 0) {
        imageBounds.w = imageWidth;
        imageBounds.h = imageHeight;
    }

    SDL_RenderCopy(renderer, texture, nullptr, &imageBounds);
}

TDT4102::Image::~Image() {
    if (texture != nullptr) {
        SDL_DestroyTexture(texture);
        texture = nullptr;
    }
}
