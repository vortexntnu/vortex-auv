#define _USE_MATH_DEFINES
#include "AnimationWindow.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#define SDL_MAIN_HANDLED
#include <SDL.h>
#include <unistd.h>

#include <array>
#include <sstream>

#include "internal/FontCache.h"
#include "internal/KeyboardKeyConverter.h"
#include "internal/nuklear_configured.h"
#include "widgets/Button.h"
static bool sdlHasBeenInitialised = false;

TDT4102::AnimationWindow::AnimationWindow(int x, int y, int width, int height, const std::string& title) {
    // Initialise SDL if it has not already been
    if (!sdlHasBeenInitialised) {
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
            throw std::runtime_error("Failed to create an AnimationWindow: The SDL backend could not be initialised.\nError details: " + std::string(SDL_GetError()));
        }
        sdlHasBeenInitialised = true;
    }

    // Open a new window
    windowHandle = SDL_CreateWindow(
        title.c_str(), x, y, width, height, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
    if (windowHandle == nullptr) {
        throw std::runtime_error("Failed to create an AnimationWindow: The SDL backend could not open a new window.\nError details: " + std::string(SDL_GetError()));
    }

    // Create a renderer
    rendererHandle = SDL_CreateRenderer(windowHandle, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (rendererHandle == nullptr) {
        throw std::runtime_error("Failed to create an AnimationWindow: The SDL backend could not create a renderer.\nError details: " + std::string(SDL_GetError()));
    }

    // Default window background colour
    SDL_SetRenderDrawColor(rendererHandle, backgroundColor.redChannel, backgroundColor.greenChannel, backgroundColor.blueChannel, backgroundColor.alphaChannel);
    SDL_RenderClear(rendererHandle);

    SDL_RendererInfo rendererInfo;
    SDL_GetRendererInfo(rendererHandle, &rendererInfo);

    std::cout << "Created an SDL renderer with name: " << rendererInfo.name << std::endl;

    context = nk_sdl_init(windowHandle, rendererHandle);
    fontCache.initialize();
    fontCache.setFont(context, Font::defaultFont, 18);
    nk_clear(context);
}

TDT4102::AnimationWindow::~AnimationWindow() {
    destroy();
}

void TDT4102::AnimationWindow::destroy() {
    this->destroyed = true;

    // Free SDL resources depending on how much ended up being initialised in the constructor
    if (rendererHandle != nullptr) {
        SDL_DestroyRenderer(rendererHandle);
        rendererHandle = nullptr;
    }
    if (windowHandle != nullptr) {
        SDL_DestroyWindow(windowHandle);
        windowHandle = nullptr;
    }
    if (context != nullptr) {
        nk_free(context);
        context = nullptr;
    }
    // Needed for MacOS
    // Window does not close unless the events are pumped
    // after requesting that
    pump_events();
}

void TDT4102::AnimationWindow::pump_events() {
    SDL_Event event;
    SDL_PumpEvents();
 
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            closeRequested = true;
        } else if (event.type == SDL_KEYDOWN) {
            KeyboardKey pressedKey = TDT4102::internal::convertSDLKeyToKeyboardKey(event.key.keysym);
            // Using [] syntax here intentionally such that new entries are created automatically if they don't exist inside the map
            currentKeyStates[pressedKey] = true;
        } else if (event.type == SDL_KEYUP) {
            KeyboardKey releasedKey = TDT4102::internal::convertSDLKeyToKeyboardKey(event.key.keysym);
            currentKeyStates[releasedKey] = false;
        } else if (event.type == SDL_MOUSEBUTTONDOWN) {
            if (event.button.button == SDL_BUTTON_LEFT) {
                currentLeftMouseButtonState = true;
            } else if (event.button.button == SDL_BUTTON_RIGHT) {
                currentRightMouseButtonState = true;
            }
        } else if (event.type == SDL_MOUSEBUTTONUP) {
            if (event.button.button == SDL_BUTTON_LEFT) {
                currentLeftMouseButtonState = false;
            } else if (event.button.button == SDL_BUTTON_RIGHT) {
                currentRightMouseButtonState = false;
            }
        } else if (event.type == SDL_MOUSEWHEEL) {
            deltaMouseWheel = event.wheel.preciseY; // The amount scrolled vertically, positive away from the user and negative toward the user
        }

        if(!destroyed) {
            nk_sdl_handle_event(&event);
        }
        
    }
}

void TDT4102::AnimationWindow::show_frame() {
    SDL_RenderPresent(rendererHandle);
    deltaMouseWheel = 0;
    nk_input_begin(context);
    pump_events();
    nk_input_end(context);
}

void TDT4102::AnimationWindow::update_gui() {
    for (Widget& widget : widgets) {
        fontCache.setFont(context, Font::arial, 18);
        if (widget.isVisible) {
            startNuklearDraw(widget.position, widget.uniqueWidgetName, widget.width, widget.height);
            widget.update(context);
            endNuklearDraw();
        }
    }
}

void TDT4102::AnimationWindow::next_frame() {
    update_gui();
    nk_sdl_render(NK_ANTI_ALIASING_ON);

    show_frame();

    // Colour must be reset as a previously drawn element may have changed the current colour
    if (!keepPreviousFrame) {
        SDL_SetRenderDrawColor(rendererHandle, backgroundColor.redChannel, backgroundColor.greenChannel, backgroundColor.blueChannel, backgroundColor.alphaChannel);
        SDL_RenderClear(rendererHandle);
    }

    // Reset window name counters
    textWindowCounter = 0;
}

bool TDT4102::AnimationWindow::should_close() const {
    return closeRequested;
}

void TDT4102::AnimationWindow::close() {
    closeRequested = true;
}

void TDT4102::AnimationWindow::wait_for_close() {
    // This forces text to render, and ensures it appears on the screenshot that will be shown perpetually
    // update_gui();
    nk_sdl_render(NK_ANTI_ALIASING_ON);

    // take a screenshot such that the window contents can be redrawn
    TDT4102::Point windowSize = getWindowDimensions();
    SDL_Surface* screenContents = SDL_CreateRGBSurface(0, windowSize.x, windowSize.y, 32, 0x00ff0000, 0x0000ff00, 0x000000ff, 0xff000000);
    SDL_RenderReadPixels(rendererHandle, NULL, SDL_PIXELFORMAT_ARGB8888, screenContents->pixels, screenContents->pitch);
    SDL_Texture* screenTexture = SDL_CreateTextureFromSurface(rendererHandle, screenContents);

    while (!should_close()) {
        next_frame();

        // Draw screen contents
        SDL_Rect imageBounds{0, 0, windowSize.x, windowSize.y};
        SDL_RenderCopy(rendererHandle, screenTexture, nullptr, &imageBounds);
    }

    // Free the screenshot when done
    SDL_DestroyTexture(screenTexture);
    SDL_FreeSurface(screenContents);

    // Force close the window
    destroy();
}

void TDT4102::AnimationWindow::wait_for(double timeSeconds) {
    usleep(int(1000000.0 * timeSeconds));
}

void TDT4102::AnimationWindow::keep_previous_frame(bool enabled) {
    this->keepPreviousFrame = enabled;
}

void TDT4102::AnimationWindow::draw_circle(TDT4102::Point centre, int radius, TDT4102::Color color, TDT4102::Color borderColour) {
    /*struct nk_rect bounds {float(centre.x - radius), float(centre.y - radius), float(2 * radius), float(2 * radius)};
    nk_color circleColour {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel};
    nk_fill_circle(nk_window_get_canvas(context), bounds, circleColour);*/

    SDL_Vertex centreVertex;
    centreVertex.position = {float(centre.x), float(centre.y)};
    centreVertex.color = {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel};
    centreVertex.tex_coord = {0, 0};
    internal::circleCoordinateBuffer.at(0) = centreVertex;

    for (int i = 1; i <= internal::SLICES_PER_CIRCLE; i++) {
        SDL_Vertex outerVertex;
        float fraction = float(i) / float(internal::SLICES_PER_CIRCLE);
        float angle = fraction * (M_PI * 2.0f);
        outerVertex.position = {float(centre.x) + (float(radius) * float(std::cos(angle))),
                                float(centre.y) + (float(radius) * float(std::sin(angle)))};
        outerVertex.color = {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel};
        outerVertex.tex_coord = {0, 0};
        internal::circleCoordinateBuffer.at(i) = outerVertex;

        // Main circle index buffer
        internal::circleIndexBuffer.at(3 * (i - 1) + 0) = 0;
        internal::circleIndexBuffer.at(3 * (i - 1) + 1) = i - 1;
        internal::circleIndexBuffer.at(3 * (i - 1) + 2) = i;

        // Border index buffer
        if (borderColour != Color::transparent) {
            internal::circleBorderBuffer.at(i - 1) = {int(outerVertex.position.x), int(outerVertex.position.y)};
        }
    }

    // Correct the first triangle which connects the first and last vertex
    internal::circleIndexBuffer.at(0) = 0;
    internal::circleIndexBuffer.at(1) = internal::SLICES_PER_CIRCLE;
    internal::circleIndexBuffer.at(2) = 1;

    internal::circleBorderBuffer.at(internal::SLICES_PER_CIRCLE) = internal::circleBorderBuffer.at(0);

    SDL_RenderGeometry(rendererHandle, nullptr, internal::circleCoordinateBuffer.data(), internal::circleCoordinateBuffer.size(),
                       internal::circleIndexBuffer.data(), internal::circleIndexBuffer.size());
    if (borderColour != Color::transparent) {
        SDL_SetRenderDrawColor(rendererHandle, borderColour.redChannel, borderColour.greenChannel, borderColour.blueChannel, borderColour.alphaChannel);
        SDL_RenderDrawLines(rendererHandle, internal::circleBorderBuffer.data(), internal::circleBorderBuffer.size());
    }
}

void TDT4102::AnimationWindow::draw_rectangle(TDT4102::Point topLeftPoint, int width, int height, TDT4102::Color color, TDT4102::Color borderColor) {
    SDL_Rect fillRect = {topLeftPoint.x, topLeftPoint.y, width, height};
    SDL_SetRenderDrawColor(rendererHandle, color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel);
    SDL_RenderFillRect(rendererHandle, &fillRect);
    if (borderColor != Color::transparent) {
        SDL_SetRenderDrawColor(rendererHandle, borderColor.redChannel, borderColor.greenChannel, borderColor.blueChannel, borderColor.alphaChannel);
        SDL_RenderDrawRect(rendererHandle, &fillRect);
    }
}

void TDT4102::AnimationWindow::draw_image(TDT4102::Point topLeftPoint, TDT4102::Image& image, int imageWidth, int imageHeight) {
    image.draw(rendererHandle, topLeftPoint, imageWidth, imageHeight);
}

void TDT4102::AnimationWindow::draw_text(TDT4102::Point topLeftPoint, std::string textToShow, TDT4102::Color color, unsigned int fontSize, TDT4102::Font font) {
    textWindowCounter++;
    std::stringstream windowName;
    windowName << "text" << textWindowCounter;
    startNuklearDraw(topLeftPoint, windowName.str(), textToShow.size()*fontSize, fontSize);
    fontCache.setFont(context, font, fontSize);
    nk_color textColour{color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel};
    nk_text_colored(context, textToShow.c_str(), textToShow.size(), NK_TEXT_ALIGN_LEFT, textColour);
    endNuklearDraw();
}

void TDT4102::AnimationWindow::draw_line(TDT4102::Point start, TDT4102::Point end, TDT4102::Color color) {
    SDL_SetRenderDrawColor(rendererHandle, color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel);
    SDL_RenderDrawLine(rendererHandle, start.x, start.y, end.x, end.y);
}

void TDT4102::AnimationWindow::draw_triangle(TDT4102::Point vertex0, TDT4102::Point vertex1,
                                             TDT4102::Point vertex2, TDT4102::Color color) {
    SDL_Vertex v0{{float(vertex0.x), float(vertex0.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    SDL_Vertex v1{{float(vertex1.x), float(vertex1.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    SDL_Vertex v2{{float(vertex2.x), float(vertex2.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    std::array<SDL_Vertex, 3> vertexArray{v0, v1, v2};
    std::array<int, 3> indexArray{0, 1, 2};
    SDL_RenderGeometry(rendererHandle, nullptr, vertexArray.data(), 3, indexArray.data(), 3);
}

void TDT4102::AnimationWindow::draw_quad(TDT4102::Point vertex0, TDT4102::Point vertex1, TDT4102::Point vertex2,
                                         TDT4102::Point vertex3, TDT4102::Color color) {
    SDL_Vertex v0{{float(vertex0.x), float(vertex0.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    SDL_Vertex v1{{float(vertex1.x), float(vertex1.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    SDL_Vertex v2{{float(vertex2.x), float(vertex2.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    SDL_Vertex v3{{float(vertex3.x), float(vertex3.y)}, {color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel}, {0, 0}};
    std::array<SDL_Vertex, 4> vertexArray{v0, v1, v2, v3};
    std::array<int, 6> indexArray{0, 1, 2, 0, 2, 3};
    SDL_RenderGeometry(rendererHandle, nullptr, vertexArray.data(), 4, indexArray.data(), 6);
}

void TDT4102::AnimationWindow::draw_arc(TDT4102::Point center, int width, int height, int start_degree, int end_degree, TDT4102::Color color) {
    if (start_degree < 0 || start_degree > 360) {
        throw std::invalid_argument("The start_degree parameter was set to " + std::to_string(start_degree) + ", but should be a value between 0 and 360");
    }
    if (end_degree < 0 || end_degree > 360) {
        throw std::invalid_argument("The end_degree parameter was set to " + std::to_string(end_degree) + ", but should be a value between 0 and 360");
    }
    if (start_degree >= end_degree) {
        std::swap(start_degree, end_degree);
    }

    float startFraction = float(start_degree) / 360.0f;
    float endFraction = float(end_degree) / 360.0f;
    float stepFraction = (endFraction - startFraction) / float(internal::SLICES_PER_CIRCLE);

    for (int i = 0; i < internal::SLICES_PER_CIRCLE; i++) {
        internal::circleBorderBuffer.at(i) = {
            center.x + int(float(width) * std::cos((startFraction + float(i) * stepFraction) * 2.0f * M_PI)),
            center.y + int(float(height) * -std::sin((startFraction + float(i) * stepFraction) * 2.0f * M_PI))};
    }

    SDL_SetRenderDrawColor(rendererHandle, color.redChannel, color.greenChannel, color.blueChannel, color.alphaChannel);
    SDL_RenderDrawLines(rendererHandle, internal::circleBorderBuffer.data(), internal::SLICES_PER_CIRCLE);
}

bool TDT4102::AnimationWindow::is_key_down(KeyboardKey key) const {
    if (currentKeyStates.count(key) == 0) {
        return false;
    }
    return currentKeyStates.at(key);
}

TDT4102::Point TDT4102::AnimationWindow::get_mouse_coordinates() const {
    int mouseX, mouseY;
    SDL_GetMouseState(&mouseX, &mouseY);
    return {mouseX, mouseY};
}

void TDT4102::AnimationWindow::add(TDT4102::Widget& widgetToAdd) {
    // Make sure not to create a copy
    widgets.emplace_back(widgetToAdd);
}

void TDT4102::AnimationWindow::show_info_dialog(const std::string& message) const {
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_INFORMATION, "Information", message.c_str(), windowHandle);
}

void TDT4102::AnimationWindow::show_error_dialog(const std::string& message) const {
    SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR, "Error", message.c_str(), windowHandle);
}

TDT4102::Point TDT4102::AnimationWindow::getWindowDimensions() const {
    TDT4102::Point dimensions;
    SDL_GetRendererOutputSize(rendererHandle, &dimensions.x, &dimensions.y);
    return dimensions;
}

int TDT4102::AnimationWindow::width() const {
    return getWindowDimensions().x;
}

int TDT4102::AnimationWindow::height() const {
    return getWindowDimensions().y;
}

bool TDT4102::AnimationWindow::is_left_mouse_button_down() const {
    return currentLeftMouseButtonState;
}

bool TDT4102::AnimationWindow::is_right_mouse_button_down() const {
    return currentRightMouseButtonState;
}

float TDT4102::AnimationWindow::get_delta_mouse_wheel() const {
    return deltaMouseWheel;
}

void TDT4102::AnimationWindow::startNuklearDraw(TDT4102::Point location, std::string uniqueWindowName, unsigned int width, unsigned int height) {
    // Make window transparent; we just want to see individual GUI elements
    struct nk_style* s = &context->style;
    nk_style_push_color(context, &s->window.background, nk_rgba(0, 0, 0, 0));
    nk_style_push_style_item(context, &s->window.fixed_background, nk_style_item_color(nk_rgba(0, 0, 0, 0)));

    // Start drawing
    TDT4102::Point windowSize = getWindowDimensions();
    // Compute a rectangle that spans the entire size of the window
    // Some padding is needed to accomplish this.
    const unsigned int drawAreaPadding = 20;

    // If no draw size was specified, use as much space as available inside the window
    struct nk_rect drawAreaSize;
    if (width == 0 && height == 0) {
        drawAreaSize = nk_rect(float(location.x), float(location.y), float(windowSize.x - location.x + drawAreaPadding), float(windowSize.y - location.y + drawAreaPadding));
    } else {
        drawAreaSize = nk_rect(float(location.x), float(location.y), float(width), float(height));
    }

    // Ensuring that all GUI elements have a reasonable minimum height
    // in order to avoid visual artefacts
    drawAreaSize.h = std::max<float>(drawAreaSize.h, 50);

    nk_begin(context, uniqueWindowName.c_str(), drawAreaSize, NK_WINDOW_BACKGROUND);

    // Subtracting a small amount of distance to account for window padding
    nk_layout_row_dynamic(context, drawAreaSize.h - 20, 1);

    // The window has now been drawn, so we reset the transparent drawing colour, as
    // GUI elements should be drawn using their normal colours
    nk_style_pop_color(context);
    nk_style_pop_style_item(context);
}

void TDT4102::AnimationWindow::endNuklearDraw() {
    nk_end(context);
}

void TDT4102::AnimationWindow::setBackgroundColor(TDT4102::Color newBackgroundColor) {
    backgroundColor = newBackgroundColor;
    SDL_SetRenderDrawColor(rendererHandle, backgroundColor.redChannel, backgroundColor.greenChannel, backgroundColor.blueChannel, backgroundColor.alphaChannel);
    SDL_RenderClear(rendererHandle);
}

void TDT4102::AnimationWindow::play_audio(TDT4102::Audio& audio, int loops) {
    audio.play(loops);
}

