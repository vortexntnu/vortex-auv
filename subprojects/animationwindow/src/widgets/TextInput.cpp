#include "widgets/TextInput.h"

void TDT4102::TextInput::update(nk_context *context) {
    struct nk_style* s = &context->style;
    nk_style_push_color(context, &s->edit.text_normal, textColor);
    nk_style_push_color(context, &s->edit.text_hover, textColor);
    nk_style_push_color(context, &s->edit.text_active, textColor);
    nk_style_push_style_item(context, &s->edit.normal, nk_style_item_color(boxColor));
    nk_style_push_style_item(context, &s->edit.hover, nk_style_item_color(boxColorHover));
    nk_style_push_style_item(context, &s->edit.active, nk_style_item_color(boxColorActive));
    nk_edit_string_zero_terminated(context, NK_EDIT_SELECTABLE | NK_EDIT_ALWAYS_INSERT_MODE | NK_EDIT_BOX | NK_EDIT_SIMPLE | NK_EDIT_MULTILINE, const_cast<char*>(contents.data()), internal::TEXT_INPUT_CHARACTER_LIMIT, nk_filter_ascii);
    
    // Detect whether any editing of the string has occurred
    // Nuklear only reports whether the text box has lost focus
    if(contents != previousContents) {
        fire();
    }
    previousContents = contents;

    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
    nk_style_pop_style_item(context);
}

TDT4102::TextInput::TextInput(TDT4102::Point location, unsigned int width, unsigned int height, std::string initialText)
    : TDT4102::Widget(location, width, height) {
    contents = initialText;
    contents.resize(internal::TEXT_INPUT_CHARACTER_LIMIT);

    previousContents = initialText;
    previousContents.resize(internal::TEXT_INPUT_CHARACTER_LIMIT);
}

std::string TDT4102::TextInput::getText() const {
    // The editing string contains a large number of zeroes at the end
    // This chops those off
    return std::string(contents.data());
}

void TDT4102::TextInput::setText(std::string updatedText) {
    contents = updatedText;
    contents.resize(internal::TEXT_INPUT_CHARACTER_LIMIT);

    previousContents = updatedText;
    previousContents.resize(internal::TEXT_INPUT_CHARACTER_LIMIT);
}