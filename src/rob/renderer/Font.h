
#ifndef H_ROB_FONT_H
#define H_ROB_FONT_H

#include "../graphics/GraphicsTypes.h"
#include "../Types.h"

namespace rob
{

    struct Glyph
    {
        // Position in texture
        int16_t m_x, m_y;
        // Size in pixels
        int16_t m_width, m_height;
        int16_t m_offsetX;
        int16_t m_offsetY;
        // Advance in x direction
        int16_t m_advance;

        uint16_t m_textureIdx;
    };

    class Font
    {
    public:
        Font();

        bool IsReady() const;

        void SetBase(uint16_t base);
        uint16_t GetBase() const;

        void SetHeight(uint16_t height);
        uint16_t GetHeight() const;

        void SetHorizontalSpacing(uint16_t spacing);
        uint16_t GetHorizontalSpacing() const;

        void SetLineSpacing(uint16_t spacing);
        uint16_t GetLineSpacing() const;

        void AddGlyph(uint32_t index, const Glyph &glyph);
        const Glyph& GetGlyph(uint32_t character) const;

        const Glyph& GetGlyphByIndex(size_t index) const;
        size_t GetGlyphCount() const;

        void AddTexture(size_t page, TextureHandle texture);
        TextureHandle GetTexture(size_t page) const;
        size_t GetTextureCount() const;

    private:
        uint16_t m_base;
        uint16_t m_height;
        uint16_t m_horiSpacing;
        uint16_t m_lineSpacing;

        // TODO: Enable dynamic glyph count (no hard coded limit).
        // The alocations should be done by FontManager.
        static const size_t MAX_GLYPHS = 256;
        Glyph m_glyph[MAX_GLYPHS];
        uint32_t m_glyphMapping[MAX_GLYPHS];
        size_t m_glyphCount;

        static const size_t MAX_TEXTURE_PAGES = 8;
        TextureHandle m_textures[MAX_TEXTURE_PAGES];
        size_t m_textureCount;

        // TODO: Add kerning support
        // int16_t **m_kerningPairs; (or something?)
    };

} // rob

#endif // H_ROB_FONT_H

