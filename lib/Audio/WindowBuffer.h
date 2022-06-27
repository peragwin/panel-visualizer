#pragma once

#include <stddef.h>
#include <vector>

namespace vuzicaudio
{
  class WindowBuffer
  {
  private:
    std::vector<float> buffer;
    size_t index = 0;

  public:
    WindowBuffer(size_t size) : buffer(size, 0.0) {}

    void push(std::vector<float> &x)
    {
      auto size = x.size();
      auto _size = buffer.size();
      if (size > _size)
      {
        Serial.println("bad window push size");
        return;
      }

      bool wrap = false;
      int en = index + size;
      if (en > _size)
      {
        en = _size;
        wrap = true;
      }

      for (int i = index; i < en; i++)
      {
        buffer[i] = x[i - index];
      }
      if (wrap)
      {
        int os = _size - index;
        for (int i = 0; i < size - os; i++)
        {
          buffer[i] = x[i + os];
        }
      }

      index = (index + size) % _size;
    }

    void get(std::vector<float> &x)
    {
      auto size = x.size();
      auto _size = buffer.size();
      if (size > _size)
      {
        Serial.println("bad window get size");
        return;
      }

      bool wrap = false;
      int st = index - size;
      int en = index;
      if (st < 0)
      {
        st = _size + st;
        en = _size;
        wrap = true;
      }

      for (int i = st; i < en; i++)
      {
        x[i - st] = buffer[i];
      }
      if (wrap)
      {
        const int os = en - st;
        for (int i = 0; i < index; i++)
        {
          x[i + os] = buffer[i];
        }
      }
    }
  };
};
