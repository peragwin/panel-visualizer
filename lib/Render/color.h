#pragma once

typedef struct {
  float red;
  float green;
  float blue;
} ColorGamut_t;
typedef struct {
  float r; // a fraction between 0 and 1
  float g; // a fraction between 0 and 1
  float b; // a fraction between 0 and 1
} Color_RGB;

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} Color_RGB8;

struct ColorRGB8 {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ColorRGB8() {}
  ColorRGB8(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b) {}

  ColorRGB8 &blend(ColorRGB8 &c, uint8_t mix) {
    auto one_mix = 256 - int(mix);
    r = (int(c.r) * int(mix) + int(r) * one_mix) >> 8;
    g = (int(c.g) * int(mix) + int(g) * one_mix) >> 8;
    b = (int(c.b) * int(mix) + int(b) * one_mix) >> 8;
    return *this;
  }
};

typedef struct {
  float h; // angle in degrees
  float s; // a fraction between 0 and 1
  float v; // a fraction between 0 and 1
} Color_HSV;

typedef struct {
  uint8_t a;
  uint8_t b;
  uint8_t g;
  uint8_t r;
} Color_ABGR;

typedef struct {
  float a;
  float b;
  float g;
  float r;
} Color_ABGRf;

// Color_RGB Color_FromHSV(ColorGamut_t *g, Color_HSV in);

// #endif
