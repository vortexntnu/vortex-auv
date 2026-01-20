#include "widgets/RadioButton.h"


TDT4102::RadioButton::RadioButton(TDT4102::Point location, unsigned int width, unsigned int height, std::string label) :
    TDT4102::Widget(location, width, height),
    label{std::move(label)} {}

void TDT4102::RadioButton::update(nk_context *context) {
    if (nk_input_has_mouse_click_down_in_rect(&context->input, NK_BUTTON_RIGHT, context->current->layout->clip, true)) {
        isSelected_ = false;
    }
    if (nk_input_has_mouse_click_down_in_rect(&context->input, NK_BUTTON_LEFT, context->current->layout->clip, true)) {
        isSelected_ = true;
    }

    struct nk_style* s = &context->style;    
    nk_style_push_color(context, &s->option.text_normal, labelColor);
    nk_style_push_color(context, &s->option.text_hover, labelColor);
    nk_style_push_color(context, &s->option.text_active, labelColor);
    nk_style_push_style_item(context, &s->option.normal, nk_style_item_color(radioColor));
    nk_style_push_style_item(context, &s->option.hover, nk_style_item_color(radioColorHover));
    nk_style_push_style_item(context, &s->option.active, nk_style_item_color(radioColorActive));

    if (nk_radio_label(context, label.data(), &isSelected_)) {
        isSelected_ = true;
        fire();
    }

    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
    
}

void TDT4102::RadioButton::setLabel(std::string newLabel) {
    label = newLabel;
}

bool TDT4102::RadioButton::isSelected() const {
    return (bool)isSelected_;
}

