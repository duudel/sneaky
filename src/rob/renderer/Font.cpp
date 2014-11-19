
#include "Font.h"
#include "../Assert.h"
#include "../Log.h"

namespace rob
{

    Font::Font()
        : m_base(0)
        , m_height(0)
        , m_horiSpacing(0)
        , m_lineSpacing(0)
        , m_glyphCount(0)
        , m_textureCount(0)
    {
        for (size_t i = 0; i < MAX_GLYPHS; i++)
        {
            m_glyphMapping[i] = 0;
            m_glyph[i].m_textureIdx = -1;
        }
    }

    bool Font::IsReady() const
    { return m_glyphCount > 0; }

    void Font::SetBase(uint16_t base)
    { m_base = base; }

    uint16_t Font::GetBase() const
    { return m_base; }

    void Font::SetHeight(uint16_t height)
    { m_height = height; }

    uint16_t Font::GetHeight() const
    { return m_height; }

    void Font::SetHorizontalSpacing(uint16_t spacing)
    { m_horiSpacing = spacing; }

    uint16_t Font::GetHorizontalSpacing() const
    { return m_horiSpacing; }

    void Font::SetLineSpacing(uint16_t spacing)
    { m_lineSpacing = spacing; }

    uint16_t Font::GetLineSpacing() const
    { return m_lineSpacing; }

    void Font::AddGlyph(uint32_t character, const Glyph &glyph)
    {
        const size_t index = m_glyphCount++;
        ROB_ASSERT(index < MAX_GLYPHS);
        m_glyphMapping[index] = character;
        m_glyph[character] = glyph;
    }

    const Glyph& Font::GetGlyph(uint32_t character) const
    {
        ROB_ASSERT(character < MAX_GLYPHS);
        if (m_glyph[character].m_textureIdx == uint16_t(-1))
            return m_glyph[uint32_t('?')];
        return m_glyph[character];
    }

    const Glyph& Font::GetGlyphByIndex(size_t index) const
    {
        ROB_ASSERT(index < m_glyphCount);
        const uint32_t character = m_glyphMapping[index];
        return GetGlyph(character);
    }

    size_t Font::GetGlyphCount() const
    { return m_glyphCount; }

    void Font::AddTexture(size_t page, TextureHandle texture)
    {
        ROB_ASSERT(page < MAX_TEXTURE_PAGES);
        m_textures[page] = texture;
        m_textureCount = (page + 1 > m_textureCount) ? page + 1 : m_textureCount;
    }

    TextureHandle Font::GetTexture(size_t page) const
    {
        ROB_ASSERT(page < MAX_TEXTURE_PAGES);
        return m_textures[page];
    }

    size_t Font::GetTextureCount() const
    { return m_textureCount; }

} // rob
