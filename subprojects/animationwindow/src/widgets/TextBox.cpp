#include "widgets/TextBox.h"

void TDT4102::TextBox::update(nk_context *context) {
    struct nk_style* s = &context->style;
    nk_style_push_color(context, &s->edit.text_normal, textColor);
    nk_style_push_color(context, &s->edit.border_color, borderColor);
    nk_style_push_style_item(context, &s->edit.normal, nk_style_item_color(boxColor));
    nk_edit_string_zero_terminated(context, NK_EDIT_READ_ONLY | NK_EDIT_MULTILINE, const_cast<char*>(contents.data()), internal::TEXT_BOX_CHARACTER_LIMIT, nk_filter_ascii);
    nk_style_pop_color(context);
    nk_style_pop_color(context);
    nk_style_pop_style_item(context);
}

TDT4102::TextBox::TextBox(TDT4102::Point location, unsigned int width, unsigned int height, std::string initialText)
    : TDT4102::Widget(location, width, height) {
    contents = initialText;
    contents.resize(internal::TEXT_BOX_CHARACTER_LIMIT);

}

std::string TDT4102::TextBox::getText() const {
    return std::string(contents.data());
}

void TDT4102::TextBox::setText(std::string updatedText) {
    contents = updatedText;
    contents.resize(internal::TEXT_BOX_CHARACTER_LIMIT);
}

