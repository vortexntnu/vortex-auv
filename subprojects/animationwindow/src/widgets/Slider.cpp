#include "widgets/Slider.h"

TDT4102::Slider::Slider(TDT4102::Point location, unsigned int width, unsigned int height, int min, int max, int initialValue, int step) 
    : TDT4102::Widget(location, width, height) {
    minValue = min;
    maxValue = max;
    value = initialValue;
    stepValue = step;
}

int TDT4102::Slider::getValue() const {
    return value;
}

void TDT4102::Slider::update(nk_context *context) {

    struct nk_style* s = &context->style;    
    nk_style_push_color(context, &s->slider.bar_normal, sliderBarColor);
    nk_style_push_color(context, &s->slider.bar_hover, sliderBarColorHover);
    nk_style_push_color(context, &s->slider.bar_active, sliderBarColorActive);
    nk_style_push_color(context, &s->slider.bar_filled, sliderBarColorFilled);
    nk_style_push_style_item(context, &s->slider.cursor_normal, nk_style_item_color(sliderCursorColor));
    nk_style_push_style_item(context, &s->slider.cursor_hover, nk_style_item_color(sliderCursorColorHover));
    nk_style_push_style_item(context, &s->slider.cursor_active, nk_style_item_color(sliderCursorColorActive));

    if (nk_slider_int(context, minValue, &value, maxValue, stepValue)) {
        fire();
    }

    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
}

