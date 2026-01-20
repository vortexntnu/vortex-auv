#include "Widget.h"

void TDT4102::Widget::setCallback(std::function<void(void)> callback) {
    this->callbackFunction = callback;
}

void TDT4102::Widget::fire() {
    if(callbackFunction != nullptr) {
        callbackFunction();
    }
}

TDT4102::Widget::Widget(TDT4102::Point location, unsigned int widgetWidth, unsigned int widgetHeight)
    : position(location), width(widgetWidth), height(widgetHeight) {
    uniqueWidgetName = "widget_" + std::to_string(internal::nextWidgetID);
    internal::nextWidgetID++;
}

void TDT4102::Widget::setVisible(bool visible) {
    isVisible = visible;
}

unsigned int TDT4102::Widget::getWidth() const { 
    return width; 
}

unsigned int TDT4102::Widget::getHeight() const { 
    return height;
}

void TDT4102::Widget::setSize(unsigned int newWidth, unsigned int newHeight) {
    width = newWidth;
    height = newHeight;
}