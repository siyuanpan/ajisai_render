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
#pragma once
#include <ajisai/ajisai.h>
#include <ajisai/core/texture2d/texture2d.h>

AJ_BEGIN

struct ShadingPoint;
struct PrimitiveIntersection;

class Material {
 public:
  virtual ~Material() = default;

  virtual ShadingPoint Shade(const PrimitiveIntersection& inct) const = 0;
};

AJISAI_API Rc<Material> CreateDiffuse(Rc<const Texture2D> albedo);
AJISAI_API Rc<Material> CreatePlastic(Rc<const Texture2D>&& albedo, float ior,
                                      float thickness, float sigma_a);
AJISAI_API Rc<Material> CreateMetal(Rc<const Texture2D> color,
                                    Rc<const Texture2D> k,
                                    Rc<const Texture2D> eta, float uroughness,
                                    float vroughness);
AJISAI_API Rc<Material> CreateMirror(Rc<const Texture2D> kr);
AJISAI_API Rc<Material> CreateDisney(
    Rc<const Texture2D> base_color, Rc<const Texture2D> subsurface,
    Rc<const Texture2D> metallic, Rc<const Texture2D> specular,
    Rc<const Texture2D> specular_tint, Rc<const Texture2D> roughness,
    Rc<const Texture2D> anisotropic, Rc<const Texture2D> sheen,
    Rc<const Texture2D> sheen_tint, Rc<const Texture2D> clearcoat,
    Rc<const Texture2D> clearcoat_gloss);
AJISAI_API Rc<Material> CreateGlass(Rc<const Texture2D> color_reflection,
                                    Rc<const Texture2D> color_refraction,
                                    Rc<const Texture2D> ior);
AJISAI_API Rc<Material> CreateNull();

AJ_END