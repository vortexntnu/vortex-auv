#pragma once

#include "Widget.h"
#include "Point.h"
#include "Color.h"
#include <string>

namespace TDT4102 {
    class CheckBox : public TDT4102::Widget {
    private:
        std::string label;
        nk_bool isSelected_ = false;
        bool lastRightMouseButtonState = false;
        bool lastLeftMouseButtonState = false;
        nk_color labelColor = nk_rgba(100, 100, 100, 255);
        nk_color checkBoxColor = nk_rgba(150, 150, 150, 255);
        nk_color checkBoxColorHover = nk_rgba(150, 150, 150, 255);
        nk_color checkBoxColorActive = nk_rgba(150, 150, 150, 255);

    protected:
        void update(nk_context* context) override;
    public:
        explicit CheckBox(TDT4102::Point location, unsigned int width, unsigned int height, std::string label);
        bool isSelected() const;
        void setLabel(std::string newLabel);
        void setLabelColor(Color newColor) {labelColor = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};};
        void setCheckBoxColor(Color newColor) {checkBoxColor = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setCheckBoxColorHover(Color newColor) {checkBoxColorHover = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
        void setCheckBoxColorActive(Color newColor) {checkBoxColorActive = nk_color{(nk_byte)newColor.redChannel, (nk_byte)newColor.greenChannel, (nk_byte)newColor.blueChannel, (nk_byte)newColor.alphaChannel};}
    };
}