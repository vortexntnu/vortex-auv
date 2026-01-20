#include "widgets/DropdownList.h"
#include <stdexcept>

TDT4102::DropdownList::DropdownList(TDT4102::Point location, unsigned int width, unsigned int height, std::vector<std::string> &initialOptions) 
    : TDT4102::Widget(location, width, height) {
    if(initialOptions.size() == 0) {
        throw std::runtime_error("The list of options must contain at least one option to choose from!");
    }
    options = initialOptions;

}

std::string TDT4102::DropdownList::getSelectedValue() const {
    return options.at(selectedIndex);
}

void TDT4102::DropdownList::update(nk_context *context) {
    if (nk_combo_begin_label(context, options.at(selectedIndex).c_str(), nk_vec2(nk_widget_width(context), 200))) {
        nk_layout_row_dynamic(context, 35, 1);
        for(unsigned int i = 0; i < options.size(); i++) {
            if (nk_combo_item_label(context, options.at(i).c_str(), NK_TEXT_LEFT)) {
                // selection changed
                selectedIndex = i;
                fire();
            }
        }
        nk_combo_end(context);
    }
}

void TDT4102::DropdownList::setOptions(std::vector<std::string> &updatedOptionsList) {
    options = updatedOptionsList;
}