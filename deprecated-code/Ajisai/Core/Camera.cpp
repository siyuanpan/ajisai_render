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
#include <Ajisai/Core/Camera.h>

namespace Ajisai::Core {

void Camera::init(float fov, float near, float far) {
  //   std::cout << world2camera << std::endl;
  //   std::cout << camera2world << std::endl;
  //   std::cout << world2camera * camera2world << std::endl;

  camera2screen = Math::Matrix4f::perspectiveProjection(
      Math::Rad<float>(fov), resolution.aspectRatio(), near, far);
  //   screen2camera = camera2screen.inverted();
  //   std::cout << camera2screen << std::endl << screen2camera << std::endl;

  screen2raster = Math::Matrix4f::scaling(
                      Math::Vector3f{resolution.x(), resolution.y(), 1.f}) *
                  Math::Matrix4f::scaling(Math::Vector3f{0.5f, -0.5f, 1.0f}) *
                  Math::Matrix4f::translation(Math::Vector3f{1.f, -1.f, 0.0f});

  //   raster2screen = screen2raster.inverted();
  //   std::cout << screen2raster << std::endl
  // << raster2screen * screen2raster << std::endl;

  raster2camera = camera2screen.inverted() * screen2raster.inverted();
  camera2raster = raster2camera.inverted();
  raster2world = camera2world * raster2camera;
  world2Raster = raster2world.inverted();

  //   std::cout << raster2camera << std::endl
  //             << camera2raster << std::endl
  //             << raster2world << std::endl
  //             << world2Raster << std::endl;
}

Ray Camera::GenerateRay(const Math::Vector2f& raster) const {
  Math::Vector3f p_film{raster.x(), raster.y(), 0.f};
  Math::Vector3f camCoord = raster2camera.transformPoint(p_film);

  Math::Vector3f origin{0.f};
  Math::Vector3f direction = camCoord.normalized();

  if (lensRadius > 0 && focus_distance > 0) {
    // TODO not PinHole model
    assert(false);
  }

  return Ray{camera2world.transformPoint(origin),
             camera2world.transformVector(direction)};
}

Ray Camera::GenerateRay(const Math::Vector2f& u1, const Math::Vector2f& u2,
                        const Math::Vector2i& raster) const {
  auto rd = lensRadius * squareToUniformDiskConcentric(u1);
  Math::Vector3f p_film{raster.x() + u2.x(), raster.y() + u2.y(), 0.f};
  Math::Vector3f camCoord = raster2camera.transformPoint(p_film);
  //   camCoord.z() = 0.f;

  Math::Vector3f origin{0.f};
  Math::Vector3f direction = camCoord.normalized();

  if (lensRadius > 0 && focus_distance > 0) {
    // TODO not PinHole model
    assert(false);
  }

  // printf(
  //     "raster (%d %d) p_film (%f %f %f) camCoord (%f %f %f) direction (%f %f
  //     "
  //     "%f) realOrigin  (%f %f %f) realDirection  (%f %f %f)\n",
  //     raster[0], raster[1], p_film[0], p_film[1], p_film[2], camCoord[0],
  //     camCoord[1], camCoord[2], direction[0], direction[1], direction[2],
  //     camera2world.transformPoint(origin)[0],
  //     camera2world.transformPoint(origin)[1],
  //     camera2world.transformPoint(origin)[2],
  //     camera2world.transformVector(direction)[0],
  //     camera2world.transformVector(direction)[1],
  //     camera2world.transformVector(direction)[2]);

  // std::cout << camera2world << std::endl
  //           << direction << std::endl
  //           << camera2world.transformVector(direction) << std::endl;
  // std::exit(1);

  return Ray{camera2world.transformPoint(origin),
             camera2world.transformVector(direction)};
}

}  // namespace Ajisai::Core