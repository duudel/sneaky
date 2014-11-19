
#ifndef H_ROB_BMF_FONT_INTERNAL_H
#define H_ROB_BMF_FONT_INTERNAL_H

namespace rob
{

    struct BmfInfoBlock
    {
        static constexpr uint8_t TYPE = 0x01;

        int16_t font_size;
        uint8_t bit_field;
        uint8_t char_set;
        uint16_t stretch_h;
        uint8_t anti_aliasing;
        uint8_t padding_up;
        uint8_t padding_right;
        uint8_t padding_down;
        uint8_t padding_left;
        uint8_t spacing_horizontal;
        uint8_t spacing_vertical;
        uint8_t outline;
    };

    struct BmfCommonBlock
    {
        static constexpr uint8_t TYPE = 0x02;

        uint16_t line_height;
        uint16_t base;
        uint16_t scale_w;
        uint16_t scale_h;
        uint16_t pages;
        uint8_t bit_field;
        uint8_t alpha_channel;
        uint8_t red_channel;
        uint8_t green_channel;
        uint8_t blue_channel;
    };

    struct BmfPageBlock
    {
        static constexpr uint8_t TYPE = 0x03;
    };

    struct BmfCharBlock
    {
        static constexpr uint8_t TYPE = 0x04;

        uint32_t id;
        uint16_t x;
        uint16_t y;
        uint16_t width;
        uint16_t height;
        int16_t offset_x;
        int16_t offset_y;
        uint16_t advance_x;
        uint8_t page;
        uint8_t channel;
    };

    struct BmfKerningPairBlock
    {
        static constexpr uint8_t TYPE = 0x05;

        uint32_t first;
        uint32_t second;
        int16_t amount;
    };

} // rob

#endif // H_ROB_BMF_FONT_INTERNAL_H

