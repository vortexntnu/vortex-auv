#pragma once

#include "Widget.h"
#include <string>
#include <vector>

namespace TDT4102 {
    class DropdownList : public TDT4102::Widget {
    private:
        std::vector<std::string> options;
        unsigned int selectedIndex = 0;
    protected:
        void update(nk_context* context) override;
    public:
        explicit DropdownList(TDT4102::Point location, unsigned int width, unsigned int height, std::vector<std::string> &initialOptions);
        std::string getSelectedValue() const;
        void setOptions(std::vector<std::string> &updatedOptionsList);
    };
}