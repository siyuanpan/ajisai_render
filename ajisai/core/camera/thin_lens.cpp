/*
Copyright 2021 Siyuan Pan <pansiyuan.cs@gmail.com>

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
*/
#include <ajisai/ajisai.h>
#include <ajisai/core/camera/camera.h>

AJ_BEGIN

class ThinLensCamera : public Camera {
 public:
  explicit ThinLensCamera(const Vector2f& resolution, const Vector3f& pos,
                          const Vector3f& look_at, const Vector3f& up,
                          float fov, float lens_radius, float focal_distance)
      : resolution_{resolution} {}

  virtual Rc<Film> CreateFilm() {
    return RcNew<Film>(Vector2i{(int)resolution_[0], (int)resolution_[1]});
  }

 private:
  Vector2f resolution_;
};

Rc<Camera> CreateThinLensCamera(const Vector2f& resolution, const Vector3f& pos,
                                const Vector3f& look_at, const Vector3f& up,
                                float fov, float lens_radius,
                                float focal_distance) {
  return RcNew<ThinLensCamera>(resolution, pos, look_at, up, fov, lens_radius,
                               focal_distance);
}

AJ_END