
#include "FontCache.h"
#include "BmfFont.internal.h"
#include "../util/StreamUtil.h"

#include "MasterCache.h"
#include "../graphics/Graphics.h"
#include "../Types.h"
#include "../Log.h"

#include <fstream>

namespace rob
{

    FontCache::FontCache(Graphics *graphics, MasterCache *cache)
        : m_graphics(graphics)
        , m_cache(cache)
    { }

    ROB_DEFINE_RESOURCE_CACHE_DTOR(FontCache)

    static bool LoadFont(std::istream &in, Font &font, MasterCache *cache);

    bool FontCache::Load(const char * const filename, Font &font)
    {
        std::ifstream in(filename, std::ios::binary);
        if (!in.is_open())
        {
            log::Error("Could not open font file ", filename);
            return false;
        }

        return LoadFont(in, font, m_cache);
    }

    void FontCache::Unload(Font font)
    {
//        for (size_t i = 0; i < font.GetTextureCount(); i++)
//            m_cache->UnloadTexture(font.GetTexture(i));
    }

    void ChangePageTextureExtension(char (&buffer)[64])
    {
        size_t pos = 0;
        for (size_t i = 0; i < 64 && buffer[i]; i++)
        {
            if (buffer[i] == '.') pos = i;
        }
        buffer[++pos] = 't';
        buffer[++pos] = 'e';
        buffer[++pos] = 'x';
        buffer[++pos] = '\0';
    }

    static bool LoadFont(std::istream &in, Font &font, MasterCache *cache)
    {
        static constexpr uint8_t BmfVersion = 3;

        if (! (ReadValue<uint8_t>(in) == 'B'
            && ReadValue<uint8_t>(in) == 'M'
            && ReadValue<uint8_t>(in) == 'F'))
        {
            log::Error("Font cache: Invalid file header.");
            return false;
        }

        const uint8_t version = ReadValue<uint8_t>(in);
        if (version != BmfVersion)
        {
            log::Error("Font cache: Wrong version. Expected ", BmfVersion, ", found ", static_cast<unsigned int>(version));
            return false;
        }

        size_t pageCount = 0;

        while (in)
        {
            const uint8_t block_type = ReadValue<uint8_t>(in);
            if (in.eof()) break;
            const uint32_t block_size = ReadValue<uint32_t>(in);

            switch (block_type)
            {
            case BmfInfoBlock::TYPE:
                {
                    const std::ios::streampos pos = in.tellg();
                    BmfInfoBlock block;
                    block.font_size         = ReadValue<int16_t>(in);
                    block.bit_field         = ReadValue<uint8_t>(in);
                    block.char_set          = ReadValue<uint8_t>(in);
                    block.stretch_h         = ReadValue<uint16_t>(in);
                    block.anti_aliasing     = ReadValue<uint8_t>(in);
                    block.padding_up        = ReadValue<uint8_t>(in);
                    block.padding_right     = ReadValue<uint8_t>(in);
                    block.padding_down      = ReadValue<uint8_t>(in);
                    block.padding_left      = ReadValue<uint8_t>(in);
                    block.spacing_horizontal = ReadValue<uint8_t>(in);
                    block.spacing_vertical  = ReadValue<uint8_t>(in);
                    block.outline           = ReadValue<uint8_t>(in);

                    // Skip font name
                    const size_t block_read_size = in.tellg() - pos;
                    const size_t block_discard_size = block_size - block_read_size;
                    in.seekg(block_discard_size, std::ios::cur);

                    font.SetLineSpacing(block.spacing_vertical);
                }
                break;

            case BmfCommonBlock::TYPE:
                {
                    BmfCommonBlock block;
                    block.line_height    = ReadValue<uint16_t>(in);
                    block.base           = ReadValue<uint16_t>(in);
                    block.scale_w        = ReadValue<uint16_t>(in);
                    block.scale_h        = ReadValue<uint16_t>(in);
                    block.pages          = ReadValue<uint16_t>(in);
                    block.bit_field      = ReadValue<uint8_t>(in);
                    block.alpha_channel  = ReadValue<uint8_t>(in);
                    block.red_channel    = ReadValue<uint8_t>(in);
                    block.green_channel  = ReadValue<uint8_t>(in);
                    block.blue_channel   = ReadValue<uint8_t>(in);
                    pageCount = block.pages;

                    font.SetBase(block.base);
                    font.SetHeight(block.line_height);
                }
                break;

            case BmfPageBlock::TYPE:
                {
                    char pageName[64];
                    size_t pageNameLen = 0;
                    const std::ios::streampos startPos = in.tellg();
                    for (size_t i = 0; i < pageCount; i++)
                    {
                        in.seekg(startPos + std::ios::streamoff(i * pageNameLen), std::ios::beg);
                        size_t len = ReadString(in, pageName);
                        if (!pageNameLen) pageNameLen = len;

//                        log::Info("FontCache: page: ", pageName, ", id: ", ResourceID(+pageName));
                        ChangePageTextureExtension(pageName);
                        // NOTE: To prevent from using the char[64] version, decay to char* by using unary +
                        const ResourceID texture(+pageName);
                        font.AddTexture(i, cache->GetTexture(texture));
//                        log::Info("FontCache: page: ", pageName, ", id: ", texture);
                    }
                }
                break;

            case BmfCharBlock::TYPE:
                {
                    const size_t characterCount = block_size / 20;
                    for (size_t i = 0; i < characterCount; i++)
                    {
                        BmfCharBlock block;
                        block.id        = ReadValue<uint32_t>(in);
                        block.x         = ReadValue<uint16_t>(in);
                        block.y         = ReadValue<uint16_t>(in);
                        block.width     = ReadValue<uint16_t>(in);
                        block.height    = ReadValue<uint16_t>(in);
                        block.offset_x  = ReadValue<int16_t>(in);
                        block.offset_y  = ReadValue<int16_t>(in);
                        block.advance_x = ReadValue<uint16_t>(in);
                        block.page      = ReadValue<uint8_t>(in);
                        block.channel   = ReadValue<uint8_t>(in);

                        Glyph glyph;
                        glyph.m_x       = block.x;
                        glyph.m_y       = block.y;
                        glyph.m_width   = block.width;
                        glyph.m_height  = block.height;
                        glyph.m_offsetX = block.offset_x;
                        glyph.m_offsetY = block.offset_y;
                        glyph.m_advance = block.advance_x;
                        glyph.m_textureIdx = block.page;

                        font.AddGlyph(block.id, glyph);
                    }
                }
                break;

            case BmfKerningPairBlock::TYPE:
                {
                    in.seekg(block_size, std::ios::cur);
//                    const size_t kerningPairs = block_size / 10;
//                    for (size_t i = 0; i < kerningPairs; i++)
//                    {
//                        BmfKerningPairBlock block;
//                        block.first = ReadValue<uint32_t>(in);
//                        block.second = ReadValue<uint32_t>(in);
//                        block.amount = ReadValue<int16_t>(in);
//                        font.AddKerning(block.first, block.second, block.amount);
//                    }
                }
                break;

            default:
                log::Error("Font cache: Invalid block type ", static_cast<unsigned int>(block_type));
                return false;
            }
        }
        return true;
    }

} // rob
