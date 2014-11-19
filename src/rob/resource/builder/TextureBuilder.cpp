
#include "TextureBuilder.h"
#include "../../Log.h"
#include "../../Types.h"
#include "../../graphics/Texture.h"

#include <FreeImage.h>

#include <fstream>

namespace rob
{

    TextureBuilder::TextureBuilder()
    {
        m_extensions.push_back(".png");
        m_extensions.push_back(".tga");
        m_newExtension = ".tex";
    }

    bool TextureBuilder::Build(const std::string &directory, const std::string &filename,
                               const std::string &destDirectory, const std::string &destFilename)
    {
        FREE_IMAGE_FORMAT format = ::FreeImage_GetFileType(filename.c_str(), 0);
        if (format == FIF_UNKNOWN)
        {
            format = ::FreeImage_GetFIFFromFilename(filename.c_str());
            if (format == FIF_UNKNOWN)
            {
                log::Error("Could not determine image file type for ", filename.c_str());
                return false;
            }
        }
        FIBITMAP *bitmap = ::FreeImage_Load(format, filename.c_str(), 0);

        if (!bitmap)
        {
            log::Error("Could not load image ", filename.c_str());
            return false;
        }

        if (::FreeImage_GetImageType(bitmap) != FIT_BITMAP)
        {
            log::Error("Invalid image type in ", filename.c_str());
            return false;
        }

        Texture::Format texFormat;

        const FREE_IMAGE_COLOR_TYPE colorType = ::FreeImage_GetColorType(bitmap);
        const size_t bpp = ::FreeImage_GetBPP(bitmap);
        switch (colorType)
        {
        case FIC_RGB:
//            log::Info("Image color type rgb");
            // For some reason rgba-png has color type FIC_RGB with bpp=32.
            if (bpp == 32)
                texFormat = Texture::FMT_RGBA;
            else
                texFormat = Texture::FMT_RGB;
            break;
        case FIC_RGBALPHA:
//            log::Info("Image color type rgba");
            texFormat = Texture::FMT_RGBA;
            break;
        default:
            log::Error("Unsupported image color type in image ", filename.c_str());
            return false;
        }

        const size_t width = ::FreeImage_GetWidth(bitmap);
        const size_t height = ::FreeImage_GetHeight(bitmap);
        const size_t bytesPerPixel = static_cast<size_t>(texFormat);
        const size_t imageSize = width * height * bytesPerPixel;

//        log::Info("Image info: ", width, "x", height, "x", bpp, ", bytespp", bytesPerPixel);

        unsigned char * const imageData = new unsigned char[imageSize];
        unsigned char *data = imageData;
        if (texFormat == Texture::FMT_RGB)
        {
            for(size_t y = 0; y < height; y++)
            {
                const BYTE *bits = FreeImage_GetScanLine(bitmap, y);
                for(size_t x = 0; x < width; x++)
                {
                    data[0] = bits[FI_RGBA_RED];
                    data[1] = bits[FI_RGBA_GREEN];
                    data[2] = bits[FI_RGBA_BLUE];

                    bits += bytesPerPixel;
                    data += bytesPerPixel;
                }
            }
        }
        else
        {
            for(size_t y = 0; y < height; y++)
            {
                const BYTE *bits = FreeImage_GetScanLine(bitmap, y);
                for(size_t x = 0; x < width; x++)
                {
                    data[0] = bits[FI_RGBA_RED];
                    data[1] = bits[FI_RGBA_GREEN];
                    data[2] = bits[FI_RGBA_BLUE];
                    data[3] = bits[FI_RGBA_ALPHA];

                    bits += bytesPerPixel;
                    data += bytesPerPixel;
                }
            }
        }

        ::FreeImage_Unload(bitmap);
        bitmap = nullptr;

        bool result = false;
        std::ofstream out(destFilename.c_str(), std::ios_base::binary);
        if (!out.is_open())
        {
            log::Error("Could not open the destination file for image ", destFilename.c_str());
        }
        else
        {
            out.write(reinterpret_cast<const char*>(&width), sizeof(size_t));
            out.write(reinterpret_cast<const char*>(&height), sizeof(size_t));
            const size_t st_format = static_cast<size_t>(texFormat);
            out.write(reinterpret_cast<const char*>(&st_format), sizeof(size_t));
            out.write(reinterpret_cast<const char*>(imageData), imageSize);

            result = true;
        }

        delete[] imageData;
        return result;
    }

} // rob
