#pragma once

#include "Widget.h"
#include "Point.h"
#include "Color.h"

namespace TDT4102 {
    class Slider : public TDT4102::Widget {
    private:
        int value;
        int minValue;
        int maxValue;
        int stepValue;
        nk_color sliderBarColor = nk_rgba(100, 100, 100, 255);
        nk_color sliderBarColorActive = nk_rgba(100, 100, 100, 255);
        nk_color sliderBarColorHover = nk_rgba(100, 100, 100, 255);
        nk_color sliderBarColorFilled = nk_rgba(100, 100, 100, 255);
        nk_color sliderCursorColor = nk_rgba(180, 180, 180, 255);
        nk_color sliderCursorColorActive = nk_rgba(180, 180, 180, 255);
        nk_color sliderCursorColorHover = nk_rgba(180, 180, 180, 255);
    protected:
        void update(nk_context* context) override;
    public:
        explicit Slider(TDT4102::Point location, unsigned int width, unsigned int height, int min = 0, int max = 100, int initialValue = 0, int step = 1);
        int getValue() const;
        void setSliderBarColor(Color newColor) {sliderBarColor = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};};
        void setSliderBarColorActive(Color newColor) {sliderBarColorActive = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setSliderBarColorHover(Color newColor) {sliderBarColorHover = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setSliderBarColorFilled(Color newColor) {sliderBarColorFilled = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setSliderCursorColor(Color newColor) {sliderCursorColor = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setSliderCursorColorHover(Color newColor) {sliderCursorColorHover = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setSliderCurserColorActive(Color newColor) {sliderCursorColorActive = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
    };
}