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
#include <ajisai/factory/creator/texture2d_creators.h>
#include <ajisai/factory/creator/helper.h>
#include <ajisai/math/color.h>

#include <string>

AJ_BEGIN

namespace {
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
}  // namespace

class ConstantCreatorImpl {
 public:
  static std::string Name() { return "constant"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    if (node["texel"].IsSequence())
      return CreateConstant2DTexture(Spectrum{node["texel"].as<Vector3f>()});
    else
      return CreateConstant2DTexture(Spectrum{node["texel"].as<float>()});
    // const auto texel = node["texel"].as<Vector3f>();

    // return CreateConstant2DTexture(Spectrum{texel});
  }
};

class ImageCreatorImpl {
 public:
  static std::string Name() { return "image"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto filename = node["filename"].as<std::string>();

    int w, h, channels;
    unsigned char* data =
        stbi_load(filename.c_str(), &w, &h, &channels, STBI_rgb);

    if (!data) {
      AJ_ERROR("load file {} error", filename);
      std::exit(1);
    }

    AJ_INFO("load file {} with : w {}, h {}, c {}", filename, w, h, channels);

    auto image = RcNew<Image<Color3<unsigned char>>>(Vector2i{w, h});

    parallel_for(
        image->Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < image->Dimension().x(); ++x) {
            int img_idx = y * w * 3 + x * 3;
            (*image)(x, y) = Color3<unsigned char>{
                data[img_idx + 0], data[img_idx + 1], data[img_idx + 2]};
          }
        },
        1024);

    stbi_image_free(data);

    return CreateImageTexture(std::move(image));
  }
};

class HdrCreatorImpl {
 public:
  static std::string Name() { return "hdr"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto filename = node["filename"].as<std::string>();

    if (filename.ends_with(".pfm")) {
      return CreateHDRTexture(ReadPFM(filename));
    }

    int w, h, channels;
    float* data = stbi_loadf(filename.c_str(), &w, &h, &channels, STBI_rgb);
    // float* data = stbi_loadf(filename.c_str(), &w, &h, &channels, 4);

    if (!data) {
      AJ_ERROR("load file {} error", filename);
      std::exit(1);
    }

    AJ_INFO("load file {} with : w {}, h {}, c {}", filename, w, h, channels);

    auto image = RcNew<RGBImage>(Vector2i{w, h});

    parallel_for(
        image->Dimension().y(),
        [&](uint32_t y, uint32_t) {
          for (int x = 0; x < image->Dimension().x(); ++x) {
            int img_idx = y * w * 3 + x * 3;
            (*image)(x, y) = Color3f{data[img_idx + 0], data[img_idx + 1],
                                     data[img_idx + 2]};
          }
        },
        1024);

    stbi_image_free(data);

    return CreateHDRTexture(std::move(image));
  }

 private:
  static int readWord(FILE* fp, char* buffer, int bufferLength) {
    int n;
    int c;

    if (bufferLength < 1) return -1;

    n = 0;
    c = fgetc(fp);
    while (c != EOF && !isWhitespace(c) && n < bufferLength) {
      buffer[n] = c;
      ++n;
      c = fgetc(fp);
    }

    if (n < bufferLength) {
      buffer[n] = '\0';
      return n;
    }

    return -1;
  }
  static constexpr bool hostLittleEndian =
#if defined(__BYTE_ORDER__)
#  if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
      true
#  elif __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
      false
#  else
#    error "__BYTE_ORDER__ defined but has unexpected value"
#  endif
#else
#  if defined(__LITTLE_ENDIAN__) || defined(__i386__) || \
      defined(__x86_64__) || defined(_WIN32) || defined(WIN32)
      true
#  elif defined(__BIG_ENDIAN__)
      false
#  elif defined(__sparc) || defined(__sparc__)
      false
#  else
#    error "Can't detect machine endian-ness at compile-time."
#  endif
#endif
      ;

#define BUFFER_SIZE 80

  static inline int isWhitespace(char c) {
    return c == ' ' || c == '\n' || c == '\t';
  }

  static Rc<RGBImage> ReadPFM(const std::string& filename) {
    float* data = nullptr;
    RGBSpectrum* rgb = nullptr;
    char buffer[BUFFER_SIZE];
    unsigned int nFloats;
    int nChannels, width, height;
    float scale;
    bool fileLittleEndian;

    FILE* fp = fopen(filename.c_str(), "rb");
    if (!fp) std::exit(1);

    if (readWord(fp, buffer, BUFFER_SIZE) == -1) std::exit(1);

    if (strcmp(buffer, "Pf") == 0)
      nChannels = 1;
    else if (strcmp(buffer, "PF") == 0)
      nChannels = 3;
    else
      std::exit(1);

    if (readWord(fp, buffer, BUFFER_SIZE) == -1) std::exit(1);
    width = atoi(buffer);
    // *xres = width;

    if (readWord(fp, buffer, BUFFER_SIZE) == -1) std::exit(1);
    height = atoi(buffer);
    // *yres = height;
    auto image = RcNew<RGBImage>(Vector2i{width, height});

    if (readWord(fp, buffer, BUFFER_SIZE) == -1) std::exit(1);
    sscanf(buffer, "%f", &scale);

    nFloats = nChannels * width * height;
    data = new float[nFloats];
    for (int y = height - 1; y >= 0; --y) {
      if (fread(&data[y * nChannels * width], sizeof(float), nChannels * width,
                fp) != nChannels * width)
        std::exit(1);
    }

    fileLittleEndian = (scale < 0.f);
    if (hostLittleEndian ^ fileLittleEndian) {
      uint8_t bytes[4];
      for (unsigned int i = 0; i < nFloats; ++i) {
        memcpy(bytes, &data[i], 4);
        std::swap(bytes[0], bytes[3]);
        std::swap(bytes[1], bytes[2]);
        memcpy(&data[i], bytes, 4);
      }
    }
    if (std::abs(scale) != 1.f)
      for (unsigned int i = 0; i < nFloats; ++i) data[i] *= std::abs(scale);

    if (nChannels == 1) {
      for (size_t j = 0; j < height; ++j) {
        for (size_t i = 0; i < width; ++i) {
          (*image)(i, j) = Color3f{data[i + j * width]};
        }
      }
    } else {
      for (size_t j = 0; j < height; ++j) {
        for (size_t i = 0; i < width; ++i) {
          int img_idx = j * width * 3 + i * 3;
          (*image)(i, j) =
              Color3f{data[img_idx + 0], data[img_idx + 1], data[img_idx + 2]};
        }
      }
    }

    delete[] data;
    fclose(fp);
    AJ_INFO("Read PFM image {} ({} x {})", filename, width, height);
    return std::move(image);
  }
};

class CheckerCreatorImpl {
 public:
  static std::string Name() { return "checker"; }

  static Rc<Texture2D> Create(const YAML::Node& node,
                              const CreateFactory& factory) {
    const auto on_color = node["on_color"].as<Vector3f>();
    const auto off_color = node["off_color"].as<Vector3f>();
    int res_u = node["res_u"].as<int>();
    int res_v = node["res_v"].as<int>();

    return CreateCheckerTexture(Spectrum{on_color}, Spectrum{off_color}, res_u,
                                res_v);
  }
};

template <class TTexture2DCreatorImpl>
concept Texture2DCreatorImpl = requires(TTexture2DCreatorImpl) {
  { TTexture2DCreatorImpl::Name() } -> std::convertible_to<std::string>;
  {
    TTexture2DCreatorImpl::Create(YAML::Node{}, CreateFactory{})
    } -> std::convertible_to<Rc<Texture2D>>;
};

template <Texture2DCreatorImpl TTexture2DCreatorImpl>
class Texture2DCreator : public TTexture2DCreatorImpl {};

using ConstantCreator = Texture2DCreator<ConstantCreatorImpl>;
using ImageCreator = Texture2DCreator<ImageCreatorImpl>;
using HdrCreator = Texture2DCreator<HdrCreatorImpl>;
using CheckerCreator = Texture2DCreator<CheckerCreatorImpl>;

void AddTexture2DFactory(Factory<Texture2D>& factory) {
  factory.Add(ConstantCreator::Name(), &ConstantCreator::Create);
  factory.Add(ImageCreator::Name(), &ImageCreator::Create);
  factory.Add(HdrCreator::Name(), &HdrCreator::Create);
  factory.Add(CheckerCreator::Name(), &CheckerCreator::Create);
}

AJ_END