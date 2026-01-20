#pragma once

#include <SDL.h>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "Color.h"
#include "Font.h"
#include "Image.h"
#include "Audio.h"
#include "KeyboardKey.h"
#include "Point.h"
#include "Widget.h"
#include "internal/FontCache.h"
#include "internal/nuklear_configured.h"
#include "internal/windows_main_fix.h"
#include "MessageType.h"

namespace TDT4102 {
// Forward declaration of Widget class
class Widget;
namespace internal {
// These variables set the rate at which this window should draw frames.

static const double activeFramesPerSecond = 60.0;
static const double activeSecondsPerFrame = 1.0 / activeFramesPerSecond;

static const double idleFramesPerSecond = 15.0;
static const double idleSecondsPerFrame = 1.0 / idleFramesPerSecond;

// Higher number of vertices means better circle approximation
// Lower number of vertices results in better speed
static const int SLICES_PER_CIRCLE = 45;
[[maybe_unused]] static std::array<SDL_Vertex, SLICES_PER_CIRCLE + 1> circleCoordinateBuffer;
[[maybe_unused]] static std::array<int, 3 * SLICES_PER_CIRCLE> circleIndexBuffer;
[[maybe_unused]] static std::array<SDL_Point, SLICES_PER_CIRCLE + 1> circleBorderBuffer;
}  // namespace internal

class AnimationWindow {
   private:
    void show_frame();
    void update_gui();
    void pump_events();
    TDT4102::Point getWindowDimensions() const;
    void startNuklearDraw(TDT4102::Point location, std::string uniqueWindowName, unsigned int width = 0, unsigned int height = 0);
    void endNuklearDraw();
    void destroy();
    bool destroyed = false;

    // If set to true, new shapes will be drawn on top of the old ones. Can create some neat effects.
    // However, note that GUI elements such as buttons will not draw themselves correctly if you use this.
    bool keepPreviousFrame = false;

    bool closeRequested = false;

    std::vector<std::reference_wrapper<TDT4102::Widget>> widgets;

    TDT4102::Color backgroundColor = TDT4102::Color::white;

    // SDL related context
    SDL_Window* windowHandle = nullptr;
    SDL_Renderer* rendererHandle = nullptr;

    // Nuklear related context
    nk_context* context = nullptr;
    TDT4102::internal::FontCache fontCache;
    unsigned int textWindowCounter = 0;

    // Input related context
    std::unordered_map<KeyboardKey, bool> currentKeyStates;
    bool currentLeftMouseButtonState = false;
    bool currentRightMouseButtonState = false;
    float deltaMouseWheel = 0;

   public:
    explicit AnimationWindow(int x = 50, int y = 50, int width = 1024, int height = 768, const std::string& title = "Animation Window");
    ~AnimationWindow();

    // When you have finished drawing a frame, call this function to display it (usually at the end of your main while loop)
    void next_frame();

    // Returns true if someone has clicked the close button of the window
    bool should_close() const;

    // See the comment above talking about the keepPreviousFrame variable :)
    void keep_previous_frame(bool enabled);

    // Run the application until someone closes the window.
    // Using this function makes sure GUI elements such as buttons are interactive and are drawn.
    void wait_for_close();

    // Close window immediately the next time next_frame() or wait_for_close() is called
    void close();

    // Pauses execution for a specified amount of time
    static void wait_for(double timeSeconds);

    // These functions should hopefully be rather self-explanatory.
    // They allow you to draw a variety of different shapes.
    void draw_circle(TDT4102::Point centre, int radius, TDT4102::Color color = TDT4102::Color::dark_blue, TDT4102::Color borderColor = TDT4102::Color::transparent);
    void draw_rectangle(TDT4102::Point topLeftPoint, int width, int height, TDT4102::Color color = TDT4102::Color::dark_green, TDT4102::Color borderColor = TDT4102::Color::transparent);
    void draw_image(TDT4102::Point topLeftPoint, TDT4102::Image& image, int imageWidth = 0, int imageHeight = 0);
    void draw_text(TDT4102::Point bottomLeftPoint, std::string textToShow, TDT4102::Color color = TDT4102::Color::black, unsigned int fontSize = 20, TDT4102::Font font = TDT4102::Font::arial);
    void draw_line(TDT4102::Point start, TDT4102::Point end, TDT4102::Color color = TDT4102::Color::black);
    void draw_triangle(TDT4102::Point vertex0, TDT4102::Point vertex1, TDT4102::Point vertex2, TDT4102::Color color = TDT4102::Color::yellow);
    void draw_quad(TDT4102::Point vertex0, TDT4102::Point vertex1, TDT4102::Point vertex2, TDT4102::Point vertex3, TDT4102::Color color = TDT4102::Color::cyan);
    void draw_arc(TDT4102::Point center, int width, int height, int start_degree, int end_degree, TDT4102::Color color = TDT4102::Color::black);

    // And these functions are for handling input
    bool is_key_down(KeyboardKey key) const;
    TDT4102::Point get_mouse_coordinates() const;
    bool is_left_mouse_button_down() const;
    bool is_right_mouse_button_down() const;

    // Add a GUI widget to the window such that it becomes visible and the user can interact with it
    void add(TDT4102::Widget& widgetToAdd);

    // Show an information dialog message to the user
    void show_info_dialog(const std::string& message) const;
    // Show an alert message to the user
    void show_error_dialog(const std::string& message) const;
    
    // Getters for the window dimensions
    int width() const;
    int height() const;

    void setBackgroundColor(TDT4102::Color newBackgroundColor);

    float get_delta_mouse_wheel() const;

    // 
    void play_audio(TDT4102::Audio& audio, int loops = 0);
};
}  // namespace TDT4102
