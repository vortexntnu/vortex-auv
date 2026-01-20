#include <iostream>
#include <cassert>
#include "internal/FontCache.h"


std::filesystem::path TDT4102::internal::FontCache::findTTFInDirectory(const std::filesystem::path &directoryToSearch, const std::string& fontFileName) {
    try {
        for(const std::filesystem::path& entryInDirectory : std::filesystem::recursive_directory_iterator{directoryToSearch}) {
            if(entryInDirectory.filename().string() == fontFileName) {
                return entryInDirectory;
            }
        }
    } catch(const std::filesystem::filesystem_error& e) {
        // Had a single case where the recursive directory iterator caused a crash
        // This is a backup strategy, even though it would not work as the regular approach due to some 
        // fonts being buried within folders on some systems
        for(const std::filesystem::path& entryInDirectory : std::filesystem::directory_iterator{directoryToSearch}) {
            if(entryInDirectory.filename().string() == fontFileName) {
                return entryInDirectory;
            }
        }
    }
    return std::filesystem::path {"nonexistent"};
}

std::filesystem::path TDT4102::internal::FontCache::findTTFFile(const std::string &fontFileName) {
    for(const std::filesystem::path& directory : TTFSearchDirectories) {
        // Only search font directories that exist on your system
        if(!std::filesystem::exists(directory)) {
            continue;
        }

        std::filesystem::path foundFile = findTTFInDirectory(directory, fontFileName);
        if(std::filesystem::exists(foundFile)) {
            return foundFile;
        }
    }
    return std::filesystem::path("DOES_NOT_EXIST");
}

void TDT4102::internal::FontCache::loadFont(nk_context *context, TDT4102::Font face, unsigned int size) {
    struct nk_font_atlas *atlas;
    struct nk_font_config config = nk_font_config(0);
    struct nk_font *font;

    if(face == Font::defaultFont) {
        // Load default fallback font
        nk_sdl_font_stash_begin(&atlas);
        font = nk_font_atlas_add_default(atlas, float(size), &config);
        nk_sdl_font_stash_end();
        nk_style_set_font(context, &font->handle);
    } else {
        // Load a font from a TTF file
        std::filesystem::path ttfFile = fontFileLocations.at(face);
        std::cout << "Found TTF file: " << ttfFile.string() << std::endl;

        nk_sdl_font_stash_begin(&atlas);
        font = nk_font_atlas_add_from_file(atlas, ttfFile.string().c_str(), float(size), &config);
        nk_sdl_font_stash_end();
    }

    // Using [] on purpose to automatically create any missing map entries
    loadedFonts[face][size] = font;
    loadedAtlases[face][size] = atlas;
}

void TDT4102::internal::FontCache::setFont(nk_context *context, TDT4102::Font face, unsigned int size) {
    // We have not seen this font face with this size before. We therefore need to load it.
    if(loadedFonts.count(face) == 0 || loadedFonts.at(face).count(size) == 0) {
        loadFont(context, face, size);
    }

    // Set the font as the main drawing font
    nk_style_load_all_cursors(context, loadedAtlases.at(face).at(size)->cursors);
    nk_style_set_font(context, &loadedFonts.at(face).at(size)->handle);
}


void TDT4102::internal::FontCache::initialize() {
    for(const std::pair<const TDT4102::Font, std::vector<std::string>> &fontFaceAlternatives : TTFFilenames) {
        bool suitableFontFound = false;
        for(const std::string& fontFaceFile : fontFaceAlternatives.second) {
            std::filesystem::path potentiallyFoundPath = findTTFFile(fontFaceFile);
            if(!std::filesystem::exists(potentiallyFoundPath)) {
                continue;
            }
            fontFileLocations[fontFaceAlternatives.first] = potentiallyFoundPath;
            // If the path exists we know it succeeded
            suitableFontFound = true;
            break;
        }
        if(!suitableFontFound) {
            throw std::runtime_error("No suitable font found for " + fontFaceAlternatives.second.at(0) + " on your system.");
        }
    }
}
