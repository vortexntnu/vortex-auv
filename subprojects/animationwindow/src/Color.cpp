#include "Color.h"

TDT4102::Color::Color(unsigned char redChannelValue, unsigned char greenChannelValue, unsigned char blueChannelValue, unsigned char transparencyValue) 
: redChannel{redChannelValue}, 
  greenChannel{greenChannelValue}, 
  blueChannel{blueChannelValue}, 
  alphaChannel{transparencyValue} {}

TDT4102::Color::Color(unsigned int hexadecimalColour) {
      // You do NOT need to understand what is going on here =)
      // This function has a lot of bit magic that is DEFINITELY not in the syllabus
      // This is not a "correct" implementation of a function such as this, but it allows you to paste in
      // a hex colour directly from a colour picker without having to account for the alpha channel.
      // This would cause the transparency to be zero
      unsigned int hexColour = hexadecimalColour;
      if(hexColour <= 0x00FFFFFF) {
          // The blue, green, and alpha channels will have been specified.
          // We move these 3 8-bit channels to the left by 8 bits, which means they become the
          // Red, green, and blue channels.
          hexColour = hexColour << 8;
          // Make the colour opaque by setting the transparency channel to 255
          hexColour |= 0xFF;
      }
    
      // Doing some bitwise magic to separate the 32-bit colour value into 4 separate 8-bit colour channels
      redChannel = (hexColour >> 24) & 0xFF;
      greenChannel = (hexColour >> 16) & 0xFF;
      blueChannel = (hexColour >> 8) & 0xFF;
      alphaChannel = (hexColour >> 0) & 0xFF;
}

bool TDT4102::Color::operator!=(Color otherColor) {
  return 
    (otherColor.redChannel != redChannel) ||
    (otherColor.greenChannel != greenChannel) ||
    (otherColor.blueChannel != blueChannel) ||
    (otherColor.alphaChannel != alphaChannel);
}

bool TDT4102::Color::operator==(Color otherColor) {
  return 
    (otherColor.redChannel == redChannel) &&
    (otherColor.greenChannel == greenChannel) &&
    (otherColor.blueChannel == blueChannel) &&
    (otherColor.alphaChannel == alphaChannel);
}
