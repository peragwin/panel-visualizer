#pragma once

#include <stddef.h>

template <size_t SIZE>
class WindowBuffer
{
private:
  float buffer[SIZE];
  size_t _size = SIZE;
  size_t index = 0;

public:
  void push(float *x, int size)
  {
    if (size > _size)
      throw "cannot push size greater than capacity";

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

  void get(float *x, int size)
  {
    if (size > _size)
      throw "cannot get size greater than capacity";

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
