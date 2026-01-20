#pragma once

#include <filesystem>
#include <vector>
#include <unordered_map>
#include "Font.h"
#include "nuklear_configured.h"

namespace TDT4102::internal {
    // The root directories containing fonts that should be searched for the TTF files
    // Paths are checked for existence before they are searched, and are thus effectively platform independent
    static const std::vector<std::filesystem::path> TTFSearchDirectories = {
        // Windows
        "C:/Windows/Fonts",

        // MacOS
        "/System/Library/Fonts",
        "/Library/Fonts",

        // Linux
        "/usr/share/fonts",
        "/usr/local/share/fonts"
    };

    static const std::unordered_map<TDT4102::Font, std::vector<std::string>> TTFFilenames {
        {Font::arial, {"Arial.ttf", "arial.ttf", "LiberationSans.ttf", "LiberationSans-Regular.ttf", "DejaVuSans.ttf"}},
        {Font::arial_bold, {"Arial_Bold.ttf", "arialb.ttf", "arialbd.ttf", "Arial Bold.ttf", "LiberationSans-Bold.ttf", "DejaVuSans-Bold.ttf"}},
        {Font::arial_bold_italic, {"Arial_Bold_Italic.ttf", "arialbi.ttf", "Arial Bold Italic.ttf", "LiberationSans-BoldItalic.ttf", "DejaVuSans-BoldOblique.ttf"}},
        {Font::arial_italic, {"Arial_Italic.ttf", "ariali.ttf", "Arial Italic.ttf", "LiberationSans-Italic.ttf", "DejaVuSans-Oblique.ttf"}},

        {Font::courier, {"Courier_New.ttf", "cour.ttf", "Courier New.ttf", "LiberationMono.ttf", "LiberationMono-Regular.ttf", "DejaVuSansMono.ttf"}},
        {Font::courier_bold, {"Courier_New_Bold.ttf", "courbd.ttf", "Courier New Bold.ttf", "LiberationMono-Bold.ttf", "DejaVuSansMono-Bold.ttf"}},
        {Font::courier_bold_italic, {"Courier_New_Bold_Italic.ttf", "courbi.ttf", "Courier New Bold Italic.ttf", "LiberationMono-BoldItalic.ttf", "DejaVuSansMono-BoldOblique.ttf"}},
        {Font::courier_italic, {"Courier_New_Italic.ttf", "couri.ttf", "Courier New Bold Italic.ttf", "LiberationMono-Italic.ttf", "DejaVuSansMono-Oblique.ttf"}},

        {Font::times, {"Times_New_Roman.ttf", "times.ttf", "Times New Roman.ttf", "LiberationSerif.ttf", "LiberationSerif-Regular.ttf", "DejaVuSerif.ttf"}},
        {Font::times_bold, {"Times_New_Roman_Bold.ttf", "timesb.ttf", "timesbd.ttf", "Times New Roman Bold.ttf", "LiberationSerif-Bold.ttf", "DejaVuSerif-Bold.ttf"}},
        {Font::times_bold_italic, {"Times_New_Roman_Bold_Italic.ttf", "timesbi.ttf", "Times New Roman Bold Italic.ttf", "LiberationSerif-BoldItalic.ttf", "DejaVuSerif-BoldItalic.ttf"}},
        {Font::times_italic, {"Times_New_Roman_Italic.ttf", "timesi.ttf", "Times New Roman Italic.ttf", "LiberationSerif-Italic.ttf", "DejaVuSerif-Italic.ttf"}}
    };

    class FontCache {
        // Map for keeping track where source TTF files are stored on disk
        std::unordered_map<Font, std::filesystem::path> fontFileLocations;

        // Maps a font face and size to a loaded font
        std::unordered_map<Font, std::unordered_map<unsigned int, nk_font*>> loadedFonts;
        std::unordered_map<Font, std::unordered_map<unsigned int, nk_font_atlas*>> loadedAtlases;

        std::filesystem::path findTTFInDirectory(const std::filesystem::path &directoryToSearch, const std::string &fontFileName);
        std::filesystem::path findTTFFile(const std::string &filename);
        void loadFont(nk_context *context, TDT4102::Font face, unsigned int size);
    public:
        void initialize();
        void setFont(nk_context* context, TDT4102::Font face, unsigned int size);
    };
}
