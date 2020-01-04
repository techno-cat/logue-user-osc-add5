/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "LCWOscWaveSource.h"

static const LCWOscWaveTable lcwSawTables[] = {
    {
        0x0000, 0x0065, 0x00C9, 0x012E, 0x0192, 0x01F6, 0x025B, 0x02BF,
        0x0323, 0x0387, 0x03EB, 0x044E, 0x04B2, 0x0515, 0x0579, 0x05DB,
        0x063E, 0x06A1, 0x0703, 0x0765, 0x07C6, 0x0828, 0x0889, 0x08EA,
        0x094A, 0x09AA, 0x0A0A, 0x0A69, 0x0AC8, 0x0B26, 0x0B84, 0x0BE2,
        0x0C3F, 0x0C9C, 0x0CF8, 0x0D53, 0x0DAF, 0x0E09, 0x0E63, 0x0EBD,
        0x0F16, 0x0F6E, 0x0FC6, 0x101D, 0x1074, 0x10C9, 0x111F, 0x1173,
        0x11C7, 0x121A, 0x126D, 0x12BF, 0x1310, 0x1360, 0x13B0, 0x13FF,
        0x144D, 0x149A, 0x14E7, 0x1533, 0x157D, 0x15C7, 0x1611, 0x1659,
        0x16A1, 0x16E7, 0x172D, 0x1772, 0x17B6, 0x17F9, 0x183B, 0x187C,
        0x18BD, 0x18FC, 0x193A, 0x1977, 0x19B4, 0x19EF, 0x1A2A, 0x1A63,
        0x1A9B, 0x1AD3, 0x1B09, 0x1B3E, 0x1B73, 0x1BA6, 0x1BD8, 0x1C09,
        0x1C39, 0x1C68, 0x1C95, 0x1CC2, 0x1CED, 0x1D18, 0x1D41, 0x1D69,
        0x1D90, 0x1DB6, 0x1DDB, 0x1DFF, 0x1E21, 0x1E42, 0x1E63, 0x1E81,
        0x1E9F, 0x1EBC, 0x1ED7, 0x1EF1, 0x1F0A, 0x1F22, 0x1F39, 0x1F4E,
        0x1F63, 0x1F76, 0x1F87, 0x1F98, 0x1FA7, 0x1FB5, 0x1FC2, 0x1FCE,
        0x1FD9, 0x1FE2, 0x1FEA, 0x1FF1, 0x1FF6, 0x1FFA, 0x1FFE, 0x1FFF,
        0x2000, 0x1FFF, 0x1FFE, 0x1FFA, 0x1FF6, 0x1FF1, 0x1FEA, 0x1FE2,
        0x1FD9, 0x1FCE, 0x1FC2, 0x1FB5, 0x1FA7, 0x1F98, 0x1F87, 0x1F76,
        0x1F63, 0x1F4E, 0x1F39, 0x1F22, 0x1F0A, 0x1EF1, 0x1ED7, 0x1EBC,
        0x1E9F, 0x1E81, 0x1E63, 0x1E42, 0x1E21, 0x1DFF, 0x1DDB, 0x1DB6,
        0x1D90, 0x1D69, 0x1D41, 0x1D18, 0x1CED, 0x1CC2, 0x1C95, 0x1C68,
        0x1C39, 0x1C09, 0x1BD8, 0x1BA6, 0x1B73, 0x1B3E, 0x1B09, 0x1AD3,
        0x1A9B, 0x1A63, 0x1A2A, 0x19EF, 0x19B4, 0x1977, 0x193A, 0x18FC,
        0x18BD, 0x187C, 0x183B, 0x17F9, 0x17B6, 0x1772, 0x172D, 0x16E7,
        0x16A1, 0x1659, 0x1611, 0x15C7, 0x157D, 0x1533, 0x14E7, 0x149A,
        0x144D, 0x13FF, 0x13B0, 0x1360, 0x1310, 0x12BF, 0x126D, 0x121A,
        0x11C7, 0x1173, 0x111F, 0x10C9, 0x1074, 0x101D, 0x0FC6, 0x0F6E,
        0x0F16, 0x0EBD, 0x0E63, 0x0E09, 0x0DAF, 0x0D53, 0x0CF8, 0x0C9C,
        0x0C3F, 0x0BE2, 0x0B84, 0x0B26, 0x0AC8, 0x0A69, 0x0A0A, 0x09AA,
        0x094A, 0x08EA, 0x0889, 0x0828, 0x07C6, 0x0765, 0x0703, 0x06A1,
        0x063E, 0x05DB, 0x0579, 0x0515, 0x04B2, 0x044E, 0x03EB, 0x0387,
        0x0323, 0x02BF, 0x025B, 0x01F6, 0x0192, 0x012E, 0x00C9, 0x0065,
        0x0000, 0xFF9B, 0xFF37, 0xFED2, 0xFE6E, 0xFE0A, 0xFDA5, 0xFD41,
        0xFCDD, 0xFC79, 0xFC15, 0xFBB2, 0xFB4E, 0xFAEB, 0xFA87, 0xFA25,
        0xF9C2, 0xF95F, 0xF8FD, 0xF89B, 0xF83A, 0xF7D8, 0xF777, 0xF716,
        0xF6B6, 0xF656, 0xF5F6, 0xF597, 0xF538, 0xF4DA, 0xF47C, 0xF41E,
        0xF3C1, 0xF364, 0xF308, 0xF2AD, 0xF251, 0xF1F7, 0xF19D, 0xF143,
        0xF0EA, 0xF092, 0xF03A, 0xEFE3, 0xEF8C, 0xEF37, 0xEEE1, 0xEE8D,
        0xEE39, 0xEDE6, 0xED93, 0xED41, 0xECF0, 0xECA0, 0xEC50, 0xEC01,
        0xEBB3, 0xEB66, 0xEB19, 0xEACD, 0xEA83, 0xEA39, 0xE9EF, 0xE9A7,
        0xE95F, 0xE919, 0xE8D3, 0xE88E, 0xE84A, 0xE807, 0xE7C5, 0xE784,
        0xE743, 0xE704, 0xE6C6, 0xE689, 0xE64C, 0xE611, 0xE5D6, 0xE59D,
        0xE565, 0xE52D, 0xE4F7, 0xE4C2, 0xE48D, 0xE45A, 0xE428, 0xE3F7,
        0xE3C7, 0xE398, 0xE36B, 0xE33E, 0xE313, 0xE2E8, 0xE2BF, 0xE297,
        0xE270, 0xE24A, 0xE225, 0xE201, 0xE1DF, 0xE1BE, 0xE19D, 0xE17F,
        0xE161, 0xE144, 0xE129, 0xE10F, 0xE0F6, 0xE0DE, 0xE0C7, 0xE0B2,
        0xE09D, 0xE08A, 0xE079, 0xE068, 0xE059, 0xE04B, 0xE03E, 0xE032,
        0xE027, 0xE01E, 0xE016, 0xE00F, 0xE00A, 0xE006, 0xE002, 0xE001,
        0xE000, 0xE001, 0xE002, 0xE006, 0xE00A, 0xE00F, 0xE016, 0xE01E,
        0xE027, 0xE032, 0xE03E, 0xE04B, 0xE059, 0xE068, 0xE079, 0xE08A,
        0xE09D, 0xE0B2, 0xE0C7, 0xE0DE, 0xE0F6, 0xE10F, 0xE129, 0xE144,
        0xE161, 0xE17F, 0xE19D, 0xE1BE, 0xE1DF, 0xE201, 0xE225, 0xE24A,
        0xE270, 0xE297, 0xE2BF, 0xE2E8, 0xE313, 0xE33E, 0xE36B, 0xE398,
        0xE3C7, 0xE3F7, 0xE428, 0xE45A, 0xE48D, 0xE4C2, 0xE4F7, 0xE52D,
        0xE565, 0xE59D, 0xE5D6, 0xE611, 0xE64C, 0xE689, 0xE6C6, 0xE704,
        0xE743, 0xE784, 0xE7C5, 0xE807, 0xE84A, 0xE88E, 0xE8D3, 0xE919,
        0xE95F, 0xE9A7, 0xE9EF, 0xEA39, 0xEA83, 0xEACD, 0xEB19, 0xEB66,
        0xEBB3, 0xEC01, 0xEC50, 0xECA0, 0xECF0, 0xED41, 0xED93, 0xEDE6,
        0xEE39, 0xEE8D, 0xEEE1, 0xEF37, 0xEF8C, 0xEFE3, 0xF03A, 0xF092,
        0xF0EA, 0xF143, 0xF19D, 0xF1F7, 0xF251, 0xF2AD, 0xF308, 0xF364,
        0xF3C1, 0xF41E, 0xF47C, 0xF4DA, 0xF538, 0xF597, 0xF5F6, 0xF656,
        0xF6B6, 0xF716, 0xF777, 0xF7D8, 0xF83A, 0xF89B, 0xF8FD, 0xF95F,
        0xF9C2, 0xFA25, 0xFA87, 0xFAEB, 0xFB4E, 0xFBB2, 0xFC15, 0xFC79,
        0xFCDD, 0xFD41, 0xFDA5, 0xFE0A, 0xFE6E, 0xFED2, 0xFF37, 0xFF9B
    },
    {
        0x0000, 0xFF9B, 0xFF37, 0xFED3, 0xFE6F, 0xFE0B, 0xFDA7, 0xFD44,
        0xFCE1, 0xFC7F, 0xFC1D, 0xFBBC, 0xFB5B, 0xFAFB, 0xFA9C, 0xFA3E,
        0xF9E1, 0xF984, 0xF929, 0xF8CE, 0xF875, 0xF81D, 0xF7C6, 0xF771,
        0xF71C, 0xF6C9, 0xF678, 0xF628, 0xF5DA, 0xF58D, 0xF541, 0xF4F8,
        0xF4B0, 0xF469, 0xF425, 0xF3E2, 0xF3A2, 0xF363, 0xF326, 0xF2EB,
        0xF2B2, 0xF27B, 0xF247, 0xF214, 0xF1E4, 0xF1B5, 0xF189, 0xF15F,
        0xF138, 0xF112, 0xF0EF, 0xF0CF, 0xF0B0, 0xF094, 0xF07B, 0xF064,
        0xF04F, 0xF03C, 0xF02C, 0xF01F, 0xF014, 0xF00B, 0xF005, 0xF001,
        0xF000, 0xF001, 0xF005, 0xF00B, 0xF014, 0xF01F, 0xF02C, 0xF03C,
        0xF04F, 0xF064, 0xF07B, 0xF094, 0xF0B0, 0xF0CF, 0xF0EF, 0xF112,
        0xF138, 0xF15F, 0xF189, 0xF1B5, 0xF1E4, 0xF214, 0xF247, 0xF27B,
        0xF2B2, 0xF2EB, 0xF326, 0xF363, 0xF3A2, 0xF3E2, 0xF425, 0xF469,
        0xF4B0, 0xF4F8, 0xF541, 0xF58D, 0xF5DA, 0xF628, 0xF678, 0xF6C9,
        0xF71C, 0xF771, 0xF7C6, 0xF81D, 0xF875, 0xF8CE, 0xF929, 0xF984,
        0xF9E1, 0xFA3E, 0xFA9C, 0xFAFB, 0xFB5B, 0xFBBC, 0xFC1D, 0xFC7F,
        0xFCE1, 0xFD44, 0xFDA7, 0xFE0B, 0xFE6F, 0xFED3, 0xFF37, 0xFF9B,
        0x0000, 0x0065, 0x00C9, 0x012D, 0x0191, 0x01F5, 0x0259, 0x02BC,
        0x031F, 0x0381, 0x03E3, 0x0444, 0x04A5, 0x0505, 0x0564, 0x05C2,
        0x061F, 0x067C, 0x06D7, 0x0732, 0x078B, 0x07E3, 0x083A, 0x088F,
        0x08E4, 0x0937, 0x0988, 0x09D8, 0x0A26, 0x0A73, 0x0ABF, 0x0B08,
        0x0B50, 0x0B97, 0x0BDB, 0x0C1E, 0x0C5E, 0x0C9D, 0x0CDA, 0x0D15,
        0x0D4E, 0x0D85, 0x0DB9, 0x0DEC, 0x0E1C, 0x0E4B, 0x0E77, 0x0EA1,
        0x0EC8, 0x0EEE, 0x0F11, 0x0F31, 0x0F50, 0x0F6C, 0x0F85, 0x0F9C,
        0x0FB1, 0x0FC4, 0x0FD4, 0x0FE1, 0x0FEC, 0x0FF5, 0x0FFB, 0x0FFF,
        0x1000, 0x0FFF, 0x0FFB, 0x0FF5, 0x0FEC, 0x0FE1, 0x0FD4, 0x0FC4,
        0x0FB1, 0x0F9C, 0x0F85, 0x0F6C, 0x0F50, 0x0F31, 0x0F11, 0x0EEE,
        0x0EC8, 0x0EA1, 0x0E77, 0x0E4B, 0x0E1C, 0x0DEC, 0x0DB9, 0x0D85,
        0x0D4E, 0x0D15, 0x0CDA, 0x0C9D, 0x0C5E, 0x0C1E, 0x0BDB, 0x0B97,
        0x0B50, 0x0B08, 0x0ABF, 0x0A73, 0x0A26, 0x09D8, 0x0988, 0x0937,
        0x08E4, 0x088F, 0x083A, 0x07E3, 0x078B, 0x0732, 0x06D7, 0x067C,
        0x061F, 0x05C2, 0x0564, 0x0505, 0x04A5, 0x0444, 0x03E3, 0x0381,
        0x031F, 0x02BC, 0x0259, 0x01F5, 0x0191, 0x012D, 0x00C9, 0x0065,
        0x0000, 0xFF9B, 0xFF37, 0xFED3, 0xFE6F, 0xFE0B, 0xFDA7, 0xFD44,
        0xFCE1, 0xFC7F, 0xFC1D, 0xFBBC, 0xFB5B, 0xFAFB, 0xFA9C, 0xFA3E,
        0xF9E1, 0xF984, 0xF929, 0xF8CE, 0xF875, 0xF81D, 0xF7C6, 0xF771,
        0xF71C, 0xF6C9, 0xF678, 0xF628, 0xF5DA, 0xF58D, 0xF541, 0xF4F8,
        0xF4B0, 0xF469, 0xF425, 0xF3E2, 0xF3A2, 0xF363, 0xF326, 0xF2EB,
        0xF2B2, 0xF27B, 0xF247, 0xF214, 0xF1E4, 0xF1B5, 0xF189, 0xF15F,
        0xF138, 0xF112, 0xF0EF, 0xF0CF, 0xF0B0, 0xF094, 0xF07B, 0xF064,
        0xF04F, 0xF03C, 0xF02C, 0xF01F, 0xF014, 0xF00B, 0xF005, 0xF001,
        0xF000, 0xF001, 0xF005, 0xF00B, 0xF014, 0xF01F, 0xF02C, 0xF03C,
        0xF04F, 0xF064, 0xF07B, 0xF094, 0xF0B0, 0xF0CF, 0xF0EF, 0xF112,
        0xF138, 0xF15F, 0xF189, 0xF1B5, 0xF1E4, 0xF214, 0xF247, 0xF27B,
        0xF2B2, 0xF2EB, 0xF326, 0xF363, 0xF3A2, 0xF3E2, 0xF425, 0xF469,
        0xF4B0, 0xF4F8, 0xF541, 0xF58D, 0xF5DA, 0xF628, 0xF678, 0xF6C9,
        0xF71C, 0xF771, 0xF7C6, 0xF81D, 0xF875, 0xF8CE, 0xF929, 0xF984,
        0xF9E1, 0xFA3E, 0xFA9C, 0xFAFB, 0xFB5B, 0xFBBC, 0xFC1D, 0xFC7F,
        0xFCE1, 0xFD44, 0xFDA7, 0xFE0B, 0xFE6F, 0xFED3, 0xFF37, 0xFF9B,
        0x0000, 0x0065, 0x00C9, 0x012D, 0x0191, 0x01F5, 0x0259, 0x02BC,
        0x031F, 0x0381, 0x03E3, 0x0444, 0x04A5, 0x0505, 0x0564, 0x05C2,
        0x061F, 0x067C, 0x06D7, 0x0732, 0x078B, 0x07E3, 0x083A, 0x088F,
        0x08E4, 0x0937, 0x0988, 0x09D8, 0x0A26, 0x0A73, 0x0ABF, 0x0B08,
        0x0B50, 0x0B97, 0x0BDB, 0x0C1E, 0x0C5E, 0x0C9D, 0x0CDA, 0x0D15,
        0x0D4E, 0x0D85, 0x0DB9, 0x0DEC, 0x0E1C, 0x0E4B, 0x0E77, 0x0EA1,
        0x0EC8, 0x0EEE, 0x0F11, 0x0F31, 0x0F50, 0x0F6C, 0x0F85, 0x0F9C,
        0x0FB1, 0x0FC4, 0x0FD4, 0x0FE1, 0x0FEC, 0x0FF5, 0x0FFB, 0x0FFF,
        0x1000, 0x0FFF, 0x0FFB, 0x0FF5, 0x0FEC, 0x0FE1, 0x0FD4, 0x0FC4,
        0x0FB1, 0x0F9C, 0x0F85, 0x0F6C, 0x0F50, 0x0F31, 0x0F11, 0x0EEE,
        0x0EC8, 0x0EA1, 0x0E77, 0x0E4B, 0x0E1C, 0x0DEC, 0x0DB9, 0x0D85,
        0x0D4E, 0x0D15, 0x0CDA, 0x0C9D, 0x0C5E, 0x0C1E, 0x0BDB, 0x0B97,
        0x0B50, 0x0B08, 0x0ABF, 0x0A73, 0x0A26, 0x09D8, 0x0988, 0x0937,
        0x08E4, 0x088F, 0x083A, 0x07E3, 0x078B, 0x0732, 0x06D7, 0x067C,
        0x061F, 0x05C2, 0x0564, 0x0505, 0x04A5, 0x0444, 0x03E3, 0x0381,
        0x031F, 0x02BC, 0x0259, 0x01F5, 0x0191, 0x012D, 0x00C9, 0x0065
    },
    {
        0x0000, 0x0032, 0x0065, 0x0097, 0x00C9, 0x00FB, 0x012D, 0x015F,
        0x0191, 0x01C2, 0x01F4, 0x0225, 0x0257, 0x0288, 0x02B8, 0x02E9,
        0x0319, 0x0349, 0x0378, 0x03A8, 0x03D6, 0x0405, 0x0433, 0x0460,
        0x048D, 0x04B9, 0x04E5, 0x0510, 0x053A, 0x0563, 0x058C, 0x05B4,
        0x05DB, 0x0601, 0x0626, 0x064A, 0x066D, 0x068E, 0x06AF, 0x06CE,
        0x06EB, 0x0708, 0x0723, 0x073C, 0x0754, 0x076A, 0x077F, 0x0791,
        0x07A2, 0x07B1, 0x07BE, 0x07C9, 0x07D2, 0x07D9, 0x07DE, 0x07E0,
        0x07E0, 0x07DE, 0x07DA, 0x07D3, 0x07CA, 0x07BE, 0x07AF, 0x079E,
        0x078B, 0x0775, 0x075C, 0x0741, 0x0722, 0x0702, 0x06DE, 0x06B8,
        0x068F, 0x0664, 0x0635, 0x0604, 0x05D1, 0x059B, 0x0562, 0x0527,
        0x04E9, 0x04A9, 0x0466, 0x0421, 0x03D9, 0x0390, 0x0344, 0x02F6,
        0x02A6, 0x0255, 0x0201, 0x01AC, 0x0155, 0x00FC, 0x00A3, 0x0047,
        0xFFEB, 0xFF8E, 0xFF2F, 0xFED0, 0xFE70, 0xFE10, 0xFDB0, 0xFD4F,
        0xFCEE, 0xFC8D, 0xFC2C, 0xFBCC, 0xFB6C, 0xFB0D, 0xFAAF, 0xFA52,
        0xF9F6, 0xF99B, 0xF942, 0xF8EA, 0xF894, 0xF841, 0xF7EF, 0xF7A0,
        0xF753, 0xF708, 0xF6C1, 0xF67C, 0xF63B, 0xF5FC, 0xF5C1, 0xF589,
        0xF555, 0xF525, 0xF4F8, 0xF4D0, 0xF4AB, 0xF48B, 0xF46E, 0xF457,
        0xF443, 0xF434, 0xF42A, 0xF424, 0xF423, 0xF426, 0xF42E, 0xF43C,
        0xF44D, 0xF464, 0xF480, 0xF4A0, 0xF4C5, 0xF4EF, 0xF51E, 0xF551,
        0xF58A, 0xF5C6, 0xF608, 0xF64E, 0xF698, 0xF6E6, 0xF739, 0xF790,
        0xF7EB, 0xF84A, 0xF8AC, 0xF913, 0xF97C, 0xF9E9, 0xFA59, 0xFACC,
        0xFB42, 0xFBBB, 0xFC36, 0xFCB3, 0xFD33, 0xFDB4, 0xFE37, 0xFEBB,
        0xFF41, 0xFFC7, 0x004F, 0x00D7, 0x015F, 0x01E8, 0x0270, 0x02F8,
        0x037F, 0x0406, 0x048C, 0x0510, 0x0593, 0x0614, 0x0693, 0x0710,
        0x078B, 0x0803, 0x0878, 0x08EA, 0x0959, 0x09C4, 0x0A2C, 0x0A90,
        0x0AF0, 0x0B4C, 0x0BA3, 0x0BF6, 0x0C44, 0x0C8D, 0x0CD1, 0x0D10,
        0x0D4A, 0x0D7F, 0x0DAE, 0x0DD7, 0x0DFB, 0x0E19, 0x0E31, 0x0E43,
        0x0E50, 0x0E56, 0x0E56, 0x0E51, 0x0E45, 0x0E34, 0x0E1C, 0x0DFE,
        0x0DDB, 0x0DB1, 0x0D82, 0x0D4D, 0x0D12, 0x0CD2, 0x0C8C, 0x0C41,
        0x0BF1, 0x0B9B, 0x0B41, 0x0AE1, 0x0A7D, 0x0A15, 0x09A8, 0x0936,
        0x08C1, 0x0848, 0x07CC, 0x074C, 0x06C8, 0x0642, 0x05B9, 0x052E,
        0x04A1, 0x0411, 0x0380, 0x02ED, 0x0258, 0x01C3, 0x012D, 0x0097,
        0x0000, 0xFF69, 0xFED3, 0xFE3D, 0xFDA8, 0xFD13, 0xFC80, 0xFBEF,
        0xFB5F, 0xFAD2, 0xFA47, 0xF9BE, 0xF938, 0xF8B4, 0xF834, 0xF7B8,
        0xF73F, 0xF6CA, 0xF658, 0xF5EB, 0xF583, 0xF51F, 0xF4BF, 0xF465,
        0xF40F, 0xF3BF, 0xF374, 0xF32E, 0xF2EE, 0xF2B3, 0xF27E, 0xF24F,
        0xF225, 0xF202, 0xF1E4, 0xF1CC, 0xF1BB, 0xF1AF, 0xF1AA, 0xF1AA,
        0xF1B0, 0xF1BD, 0xF1CF, 0xF1E7, 0xF205, 0xF229, 0xF252, 0xF281,
        0xF2B6, 0xF2F0, 0xF32F, 0xF373, 0xF3BC, 0xF40A, 0xF45D, 0xF4B4,
        0xF510, 0xF570, 0xF5D4, 0xF63C, 0xF6A7, 0xF716, 0xF788, 0xF7FD,
        0xF875, 0xF8F0, 0xF96D, 0xF9EC, 0xFA6D, 0xFAF0, 0xFB74, 0xFBFA,
        0xFC81, 0xFD08, 0xFD90, 0xFE18, 0xFEA1, 0xFF29, 0xFFB1, 0x0039,
        0x00BF, 0x0145, 0x01C9, 0x024C, 0x02CD, 0x034D, 0x03CA, 0x0445,
        0x04BE, 0x0534, 0x05A7, 0x0617, 0x0684, 0x06ED, 0x0754, 0x07B6,
        0x0815, 0x0870, 0x08C7, 0x091A, 0x0968, 0x09B2, 0x09F8, 0x0A3A,
        0x0A76, 0x0AAF, 0x0AE2, 0x0B11, 0x0B3B, 0x0B60, 0x0B80, 0x0B9C,
        0x0BB3, 0x0BC4, 0x0BD2, 0x0BDA, 0x0BDD, 0x0BDC, 0x0BD6, 0x0BCC,
        0x0BBD, 0x0BA9, 0x0B92, 0x0B75, 0x0B55, 0x0B30, 0x0B08, 0x0ADB,
        0x0AAB, 0x0A77, 0x0A3F, 0x0A04, 0x09C5, 0x0984, 0x093F, 0x08F8,
        0x08AD, 0x0860, 0x0811, 0x07BF, 0x076C, 0x0716, 0x06BE, 0x0665,
        0x060A, 0x05AE, 0x0551, 0x04F3, 0x0494, 0x0434, 0x03D4, 0x0373,
        0x0312, 0x02B1, 0x0250, 0x01F0, 0x0190, 0x0130, 0x00D1, 0x0072,
        0x0015, 0xFFB9, 0xFF5D, 0xFF04, 0xFEAB, 0xFE54, 0xFDFF, 0xFDAB,
        0xFD5A, 0xFD0A, 0xFCBC, 0xFC70, 0xFC27, 0xFBDF, 0xFB9A, 0xFB57,
        0xFB17, 0xFAD9, 0xFA9E, 0xFA65, 0xFA2F, 0xF9FC, 0xF9CB, 0xF99C,
        0xF971, 0xF948, 0xF922, 0xF8FE, 0xF8DE, 0xF8BF, 0xF8A4, 0xF88B,
        0xF875, 0xF862, 0xF851, 0xF842, 0xF836, 0xF82D, 0xF826, 0xF822,
        0xF820, 0xF820, 0xF822, 0xF827, 0xF82E, 0xF837, 0xF842, 0xF84F,
        0xF85E, 0xF86F, 0xF881, 0xF896, 0xF8AC, 0xF8C4, 0xF8DD, 0xF8F8,
        0xF915, 0xF932, 0xF951, 0xF972, 0xF993, 0xF9B6, 0xF9DA, 0xF9FF,
        0xFA25, 0xFA4C, 0xFA74, 0xFA9D, 0xFAC6, 0xFAF0, 0xFB1B, 0xFB47,
        0xFB73, 0xFBA0, 0xFBCD, 0xFBFB, 0xFC2A, 0xFC58, 0xFC88, 0xFCB7,
        0xFCE7, 0xFD17, 0xFD48, 0xFD78, 0xFDA9, 0xFDDB, 0xFE0C, 0xFE3E,
        0xFE6F, 0xFEA1, 0xFED3, 0xFF05, 0xFF37, 0xFF69, 0xFF9B, 0xFFCE
    },
    {
        0x0000, 0x0011, 0x0021, 0x0032, 0x0042, 0x0051, 0x0060, 0x006D,
        0x007A, 0x0086, 0x0091, 0x009A, 0x00A2, 0x00A9, 0x00AE, 0x00B2,
        0x00B4, 0x00B4, 0x00B3, 0x00B0, 0x00AB, 0x00A5, 0x009D, 0x0093,
        0x0088, 0x007B, 0x006D, 0x005D, 0x004C, 0x0039, 0x0026, 0x0011,
        0xFFFB, 0xFFE5, 0xFFCE, 0xFFB6, 0xFF9E, 0xFF85, 0xFF6C, 0xFF53,
        0xFF3B, 0xFF22, 0xFF09, 0xFEF1, 0xFEDA, 0xFEC3, 0xFEAD, 0xFE98,
        0xFE84, 0xFE70, 0xFE5E, 0xFE4D, 0xFE3E, 0xFE2F, 0xFE23, 0xFE17,
        0xFE0D, 0xFE05, 0xFDFE, 0xFDF8, 0xFDF4, 0xFDF2, 0xFDF1, 0xFDF2,
        0xFDF4, 0xFDF7, 0xFDFC, 0xFE03, 0xFE0B, 0xFE14, 0xFE1E, 0xFE2A,
        0xFE37, 0xFE45, 0xFE54, 0xFE64, 0xFE75, 0xFE87, 0xFE9A, 0xFEAE,
        0xFEC2, 0xFED8, 0xFEEE, 0xFF05, 0xFF1D, 0xFF35, 0xFF4F, 0xFF69,
        0xFF83, 0xFF9F, 0xFFBB, 0xFFD7, 0xFFF5, 0x0013, 0x0032, 0x0051,
        0x0072, 0x0093, 0x00B5, 0x00D7, 0x00FA, 0x011E, 0x0143, 0x0168,
        0x018E, 0x01B4, 0x01DB, 0x0203, 0x022B, 0x0253, 0x027C, 0x02A4,
        0x02CD, 0x02F6, 0x031F, 0x0347, 0x036F, 0x0396, 0x03BC, 0x03E1,
        0x0406, 0x0428, 0x0449, 0x0468, 0x0486, 0x04A0, 0x04B9, 0x04CE,
        0x04E0, 0x04EF, 0x04FB, 0x0503, 0x0506, 0x0506, 0x0501, 0x04F8,
        0x04E9, 0x04D6, 0x04BD, 0x049F, 0x047C, 0x0453, 0x0425, 0x03F1,
        0x03B7, 0x0378, 0x0332, 0x02E8, 0x0298, 0x0242, 0x01E7, 0x0188,
        0x0123, 0x00BA, 0x004D, 0xFFDC, 0xFF67, 0xFEEF, 0xFE75, 0xFDF8,
        0xFD79, 0xFCF9, 0xFC78, 0xFBF7, 0xFB76, 0xFAF7, 0xFA78, 0xF9FC,
        0xF982, 0xF90C, 0xF89A, 0xF82C, 0xF7C4, 0xF761, 0xF705, 0xF6AF,
        0xF661, 0xF61C, 0xF5DF, 0xF5AB, 0xF580, 0xF55F, 0xF549, 0xF53E,
        0xF53D, 0xF548, 0xF55E, 0xF580, 0xF5AD, 0xF5E6, 0xF62B, 0xF67B,
        0xF6D7, 0xF73F, 0xF7B1, 0xF82E, 0xF8B6, 0xF947, 0xF9E3, 0xFA87,
        0xFB33, 0xFBE8, 0xFCA3, 0xFD65, 0xFE2D, 0xFEF9, 0xFFCA, 0x009D,
        0x0173, 0x024A, 0x0322, 0x03F9, 0x04CE, 0x05A1, 0x0671, 0x073C,
        0x0802, 0x08C1, 0x0979, 0x0A29, 0x0AD1, 0x0B6E, 0x0C00, 0x0C87,
        0x0D03, 0x0D71, 0x0DD2, 0x0E25, 0x0E6A, 0x0EA0, 0x0EC6, 0x0EDE,
        0x0EE5, 0x0EDD, 0x0EC5, 0x0E9D, 0x0E66, 0x0E1F, 0x0DC8, 0x0D62,
        0x0CEE, 0x0C6B, 0x0BDB, 0x0B3D, 0x0A93, 0x09DE, 0x091D, 0x0852,
        0x077D, 0x06A1, 0x05BC, 0x04D2, 0x03E2, 0x02ED, 0x01F5, 0x00FB,
        0x0000, 0xFF05, 0xFE0B, 0xFD13, 0xFC1E, 0xFB2E, 0xFA44, 0xF95F,
        0xF883, 0xF7AE, 0xF6E3, 0xF622, 0xF56D, 0xF4C3, 0xF425, 0xF395,
        0xF312, 0xF29E, 0xF238, 0xF1E1, 0xF19A, 0xF163, 0xF13B, 0xF123,
        0xF11B, 0xF122, 0xF13A, 0xF160, 0xF196, 0xF1DB, 0xF22E, 0xF28F,
        0xF2FD, 0xF379, 0xF400, 0xF492, 0xF52F, 0xF5D7, 0xF687, 0xF73F,
        0xF7FE, 0xF8C4, 0xF98F, 0xFA5F, 0xFB32, 0xFC07, 0xFCDE, 0xFDB6,
        0xFE8D, 0xFF63, 0x0036, 0x0107, 0x01D3, 0x029B, 0x035D, 0x0418,
        0x04CD, 0x0579, 0x061D, 0x06B9, 0x074A, 0x07D2, 0x084F, 0x08C1,
        0x0929, 0x0985, 0x09D5, 0x0A1A, 0x0A53, 0x0A80, 0x0AA2, 0x0AB8,
        0x0AC3, 0x0AC2, 0x0AB7, 0x0AA1, 0x0A80, 0x0A55, 0x0A21, 0x09E4,
        0x099F, 0x0951, 0x08FB, 0x089F, 0x083C, 0x07D4, 0x0766, 0x06F4,
        0x067E, 0x0604, 0x0588, 0x0509, 0x048A, 0x0409, 0x0388, 0x0307,
        0x0287, 0x0208, 0x018B, 0x0111, 0x0099, 0x0024, 0xFFB3, 0xFF46,
        0xFEDD, 0xFE78, 0xFE19, 0xFDBE, 0xFD68, 0xFD18, 0xFCCE, 0xFC88,
        0xFC49, 0xFC0F, 0xFBDB, 0xFBAD, 0xFB84, 0xFB61, 0xFB43, 0xFB2A,
        0xFB17, 0xFB08, 0xFAFF, 0xFAFA, 0xFAFA, 0xFAFD, 0xFB05, 0xFB11,
        0xFB20, 0xFB32, 0xFB47, 0xFB60, 0xFB7A, 0xFB98, 0xFBB7, 0xFBD8,
        0xFBFA, 0xFC1F, 0xFC44, 0xFC6A, 0xFC91, 0xFCB9, 0xFCE1, 0xFD0A,
        0xFD33, 0xFD5C, 0xFD84, 0xFDAD, 0xFDD5, 0xFDFD, 0xFE25, 0xFE4C,
        0xFE72, 0xFE98, 0xFEBD, 0xFEE2, 0xFF06, 0xFF29, 0xFF4B, 0xFF6D,
        0xFF8E, 0xFFAF, 0xFFCE, 0xFFED, 0x000B, 0x0029, 0x0045, 0x0061,
        0x007D, 0x0097, 0x00B1, 0x00CB, 0x00E3, 0x00FB, 0x0112, 0x0128,
        0x013E, 0x0152, 0x0166, 0x0179, 0x018B, 0x019C, 0x01AC, 0x01BB,
        0x01C9, 0x01D6, 0x01E2, 0x01EC, 0x01F5, 0x01FD, 0x0204, 0x0209,
        0x020C, 0x020E, 0x020F, 0x020E, 0x020C, 0x0208, 0x0202, 0x01FB,
        0x01F3, 0x01E9, 0x01DD, 0x01D1, 0x01C2, 0x01B3, 0x01A2, 0x0190,
        0x017C, 0x0168, 0x0153, 0x013D, 0x0126, 0x010F, 0x00F7, 0x00DE,
        0x00C5, 0x00AD, 0x0094, 0x007B, 0x0062, 0x004A, 0x0032, 0x001B,
        0x0005, 0xFFEF, 0xFFDA, 0xFFC7, 0xFFB4, 0xFFA3, 0xFF93, 0xFF85,
        0xFF78, 0xFF6D, 0xFF63, 0xFF5B, 0xFF55, 0xFF50, 0xFF4D, 0xFF4C,
        0xFF4C, 0xFF4E, 0xFF52, 0xFF57, 0xFF5E, 0xFF66, 0xFF6F, 0xFF7A,
        0xFF86, 0xFF93, 0xFFA0, 0xFFAF, 0xFFBE, 0xFFCE, 0xFFDF, 0xFFEF
    },
    {
        0x0000, 0xFFE5, 0xFFCB, 0xFFB1, 0xFF98, 0xFF80, 0xFF69, 0xFF54,
        0xFF41, 0xFF2F, 0xFF20, 0xFF13, 0xFF08, 0xFF00, 0xFEF9, 0xFEF6,
        0xFEF4, 0xFEF5, 0xFEF7, 0xFEFC, 0xFF03, 0xFF0B, 0xFF15, 0xFF20,
        0xFF2C, 0xFF39, 0xFF47, 0xFF56, 0xFF65, 0xFF75, 0xFF84, 0xFF94,
        0xFFA3, 0xFFB2, 0xFFC1, 0xFFCF, 0xFFDD, 0xFFEB, 0xFFF8, 0x0004,
        0x0010, 0x001B, 0x0026, 0x0031, 0x003B, 0x0044, 0x004D, 0x0056,
        0x005E, 0x0065, 0x006D, 0x0073, 0x007A, 0x007F, 0x0084, 0x0089,
        0x008D, 0x008F, 0x0092, 0x0093, 0x0093, 0x0092, 0x0090, 0x008D,
        0x0089, 0x0084, 0x007E, 0x0077, 0x006E, 0x0065, 0x005B, 0x0050,
        0x0045, 0x0039, 0x002E, 0x0022, 0x0016, 0x000A, 0x0000, 0xFFF6,
        0xFFED, 0xFFE5, 0xFFDE, 0xFFD9, 0xFFD6, 0xFFD5, 0xFFD5, 0xFFD7,
        0xFFDB, 0xFFE1, 0xFFE9, 0xFFF2, 0xFFFD, 0x0009, 0x0016, 0x0025,
        0x0033, 0x0042, 0x0051, 0x0060, 0x006E, 0x007B, 0x0086, 0x0090,
        0x0098, 0x009D, 0x00A0, 0x00A1, 0x009E, 0x0098, 0x0090, 0x0084,
        0x0075, 0x0063, 0x004F, 0x0037, 0x001D, 0x0001, 0xFFE3, 0xFFC4,
        0xFFA3, 0xFF82, 0xFF60, 0xFF3E, 0xFF1C, 0xFEFB, 0xFEDC, 0xFEBE,
        0xFEA2, 0xFE88, 0xFE71, 0xFE5D, 0xFE4C, 0xFE3E, 0xFE34, 0xFE2D,
        0xFE2A, 0xFE2A, 0xFE2E, 0xFE35, 0xFE3F, 0xFE4D, 0xFE5E, 0xFE72,
        0xFE89, 0xFEA3, 0xFEBF, 0xFEDD, 0xFEFE, 0xFF21, 0xFF46, 0xFF6C,
        0xFF94, 0xFFBF, 0xFFEA, 0x0018, 0x0046, 0x0077, 0x00A9, 0x00DC,
        0x0111, 0x0147, 0x017E, 0x01B7, 0x01F0, 0x022A, 0x0265, 0x029F,
        0x02D9, 0x0313, 0x034B, 0x0381, 0x03B4, 0x03E4, 0x0410, 0x0436,
        0x0457, 0x0471, 0x0483, 0x048D, 0x048D, 0x0483, 0x046D, 0x044C,
        0x041E, 0x03E4, 0x039B, 0x0346, 0x02E2, 0x0272, 0x01F4, 0x0169,
        0x00D2, 0x0031, 0xFF85, 0xFED2, 0xFE17, 0xFD57, 0xFC94, 0xFBD1,
        0xFB0E, 0xFA4F, 0xF996, 0xF8E6, 0xF840, 0xF7A8, 0xF721, 0xF6AB,
        0xF64A, 0xF600, 0xF5CE, 0xF5B7, 0xF5BB, 0xF5DB, 0xF619, 0xF674,
        0xF6ED, 0xF783, 0xF835, 0xF903, 0xF9EB, 0xFAEA, 0xFBFF, 0xFD27,
        0xFE5E, 0xFFA3, 0x00F0, 0x0243, 0x0397, 0x04E9, 0x0635, 0x0776,
        0x08A8, 0x09C8, 0x0AD2, 0x0BC2, 0x0C95, 0x0D49, 0x0DD9, 0x0E45,
        0x0E8A, 0x0EA7, 0x0E9C, 0x0E67, 0x0E0A, 0x0D84, 0x0CD8, 0x0C06,
        0x0B10, 0x09FB, 0x08C7, 0x077A, 0x0617, 0x04A1, 0x031E, 0x0191,
        0x0000, 0xFE6F, 0xFCE2, 0xFB5F, 0xF9E9, 0xF886, 0xF739, 0xF605,
        0xF4F0, 0xF3FA, 0xF328, 0xF27C, 0xF1F6, 0xF199, 0xF164, 0xF159,
        0xF176, 0xF1BB, 0xF227, 0xF2B7, 0xF36B, 0xF43E, 0xF52E, 0xF638,
        0xF758, 0xF88A, 0xF9CB, 0xFB17, 0xFC69, 0xFDBD, 0xFF10, 0x005D,
        0x01A2, 0x02D9, 0x0401, 0x0516, 0x0615, 0x06FD, 0x07CB, 0x087D,
        0x0913, 0x098C, 0x09E7, 0x0A25, 0x0A45, 0x0A49, 0x0A32, 0x0A00,
        0x09B6, 0x0955, 0x08DF, 0x0858, 0x07C0, 0x071A, 0x066A, 0x05B1,
        0x04F2, 0x042F, 0x036C, 0x02A9, 0x01E9, 0x012E, 0x007B, 0xFFCF,
        0xFF2E, 0xFE97, 0xFE0C, 0xFD8E, 0xFD1E, 0xFCBA, 0xFC65, 0xFC1C,
        0xFBE2, 0xFBB4, 0xFB93, 0xFB7D, 0xFB73, 0xFB73, 0xFB7D, 0xFB8F,
        0xFBA9, 0xFBCA, 0xFBF0, 0xFC1C, 0xFC4C, 0xFC7F, 0xFCB5, 0xFCED,
        0xFD27, 0xFD61, 0xFD9B, 0xFDD6, 0xFE10, 0xFE49, 0xFE82, 0xFEB9,
        0xFEEF, 0xFF24, 0xFF57, 0xFF89, 0xFFBA, 0xFFE8, 0x0016, 0x0041,
        0x006C, 0x0094, 0x00BA, 0x00DF, 0x0102, 0x0123, 0x0141, 0x015D,
        0x0177, 0x018E, 0x01A2, 0x01B3, 0x01C1, 0x01CB, 0x01D2, 0x01D6,
        0x01D6, 0x01D3, 0x01CC, 0x01C2, 0x01B4, 0x01A3, 0x018F, 0x0178,
        0x015E, 0x0142, 0x0124, 0x0105, 0x00E4, 0x00C2, 0x00A0, 0x007E,
        0x005D, 0x003C, 0x001D, 0xFFFF, 0xFFE3, 0xFFC9, 0xFFB1, 0xFF9D,
        0xFF8B, 0xFF7C, 0xFF70, 0xFF68, 0xFF62, 0xFF5F, 0xFF60, 0xFF63,
        0xFF68, 0xFF70, 0xFF7A, 0xFF85, 0xFF92, 0xFFA0, 0xFFAF, 0xFFBE,
        0xFFCD, 0xFFDB, 0xFFEA, 0xFFF7, 0x0003, 0x000E, 0x0017, 0x001F,
        0x0025, 0x0029, 0x002B, 0x002B, 0x002A, 0x0027, 0x0022, 0x001B,
        0x0013, 0x000A, 0x0000, 0xFFF6, 0xFFEA, 0xFFDE, 0xFFD2, 0xFFC7,
        0xFFBB, 0xFFB0, 0xFFA5, 0xFF9B, 0xFF92, 0xFF89, 0xFF82, 0xFF7C,
        0xFF77, 0xFF73, 0xFF70, 0xFF6E, 0xFF6D, 0xFF6D, 0xFF6E, 0xFF71,
        0xFF73, 0xFF77, 0xFF7C, 0xFF81, 0xFF86, 0xFF8D, 0xFF93, 0xFF9B,
        0xFFA2, 0xFFAA, 0xFFB3, 0xFFBC, 0xFFC5, 0xFFCF, 0xFFDA, 0xFFE5,
        0xFFF0, 0xFFFC, 0x0008, 0x0015, 0x0023, 0x0031, 0x003F, 0x004E,
        0x005D, 0x006C, 0x007C, 0x008B, 0x009B, 0x00AA, 0x00B9, 0x00C7,
        0x00D4, 0x00E0, 0x00EB, 0x00F5, 0x00FD, 0x0104, 0x0109, 0x010B,
        0x010C, 0x010A, 0x0107, 0x0100, 0x00F8, 0x00ED, 0x00E0, 0x00D1,
        0x00BF, 0x00AC, 0x0097, 0x0080, 0x0068, 0x004F, 0x0035, 0x001B
    },
    {
        0x0000, 0x0010, 0x001F, 0x002D, 0x003A, 0x0045, 0x004F, 0x0056,
        0x005C, 0x005F, 0x0060, 0x005F, 0x005C, 0x0058, 0x0053, 0x004C,
        0x0045, 0x003D, 0x0035, 0x002D, 0x0024, 0x001C, 0x0015, 0x000D,
        0x0006, 0xFFFF, 0xFFF9, 0xFFF2, 0xFFEC, 0xFFE6, 0xFFE1, 0xFFDC,
        0xFFD8, 0xFFD4, 0xFFD1, 0xFFCF, 0xFFCD, 0xFFCD, 0xFFCE, 0xFFD1,
        0xFFD4, 0xFFD8, 0xFFDE, 0xFFE4, 0xFFEA, 0xFFF1, 0xFFF7, 0xFFFD,
        0x0002, 0x0006, 0x0008, 0x0009, 0x0009, 0x0007, 0x0003, 0xFFFE,
        0xFFF8, 0xFFF2, 0xFFEA, 0xFFE3, 0xFFDD, 0xFFD7, 0xFFD2, 0xFFCF,
        0xFFCD, 0xFFCE, 0xFFD0, 0xFFD4, 0xFFDB, 0xFFE2, 0xFFEC, 0xFFF6,
        0x0002, 0x000E, 0x001A, 0x0026, 0x0031, 0x003B, 0x0045, 0x004D,
        0x0054, 0x005A, 0x005F, 0x0062, 0x0064, 0x0065, 0x0065, 0x0064,
        0x0061, 0x005E, 0x0059, 0x0053, 0x004B, 0x0042, 0x0037, 0x002B,
        0x001D, 0x000D, 0xFFFC, 0xFFEA, 0xFFD6, 0xFFC3, 0xFFAF, 0xFF9C,
        0xFF8A, 0xFF79, 0xFF6B, 0xFF5F, 0xFF57, 0xFF52, 0xFF51, 0xFF55,
        0xFF5C, 0xFF67, 0xFF75, 0xFF86, 0xFF9A, 0xFFB0, 0xFFC7, 0xFFDF,
        0xFFF6, 0x000D, 0x0021, 0x0034, 0x0045, 0x0053, 0x005E, 0x0066,
        0x006C, 0x006F, 0x0070, 0x0070, 0x006E, 0x006A, 0x0066, 0x0061,
        0x005C, 0x0057, 0x0051, 0x004B, 0x0045, 0x003F, 0x0038, 0x0031,
        0x0029, 0x0020, 0x0018, 0x000F, 0x0006, 0xFFFD, 0xFFF6, 0xFFEF,
        0xFFEB, 0xFFE8, 0xFFE8, 0xFFEA, 0xFFEF, 0xFFF7, 0x0001, 0x000D,
        0x001A, 0x0027, 0x0035, 0x0040, 0x0049, 0x004E, 0x004E, 0x0049,
        0x003E, 0x002D, 0x0015, 0xFFF7, 0xFFD3, 0xFFAA, 0xFF7E, 0xFF50,
        0xFF21, 0xFEF4, 0xFECA, 0xFEA5, 0xFE87, 0xFE70, 0xFE62, 0xFE5F,
        0xFE65, 0xFE77, 0xFE92, 0xFEB7, 0xFEE6, 0xFF1C, 0xFF58, 0xFF9A,
        0xFFE1, 0x002A, 0x0076, 0x00C2, 0x010F, 0x015B, 0x01A6, 0x01F1,
        0x0239, 0x0280, 0x02C3, 0x0303, 0x033F, 0x0373, 0x03A0, 0x03C2,
        0x03D6, 0x03DA, 0x03CA, 0x03A3, 0x0363, 0x0306, 0x028B, 0x01F1,
        0x0137, 0x005F, 0xFF6B, 0xFE60, 0xFD43, 0xFC1A, 0xFAEF, 0xF9CB,
        0xF8B8, 0xF7C2, 0xF6F2, 0xF654, 0xF5F1, 0xF5D2, 0xF5FE, 0xF678,
        0xF742, 0xF85D, 0xF9C3, 0xFB6D, 0xFD53, 0xFF67, 0x019A, 0x03DB,
        0x0619, 0x0840, 0x0A3D, 0x0BFE, 0x0D72, 0x0E8B, 0x0F3C, 0x0F7D,
        0x0F4A, 0x0EA0, 0x0D84, 0x0BFC, 0x0A14, 0x07DB, 0x0561, 0x02BC,
        0x0000, 0xFD44, 0xFA9F, 0xF825, 0xF5EC, 0xF404, 0xF27C, 0xF160,
        0xF0B6, 0xF083, 0xF0C4, 0xF175, 0xF28E, 0xF402, 0xF5C3, 0xF7C0,
        0xF9E7, 0xFC25, 0xFE66, 0x0099, 0x02AD, 0x0493, 0x063D, 0x07A3,
        0x08BE, 0x0988, 0x0A02, 0x0A2E, 0x0A0F, 0x09AC, 0x090E, 0x083E,
        0x0748, 0x0635, 0x0511, 0x03E6, 0x02BD, 0x01A0, 0x0095, 0xFFA1,
        0xFEC9, 0xFE0F, 0xFD75, 0xFCFA, 0xFC9D, 0xFC5D, 0xFC36, 0xFC26,
        0xFC2A, 0xFC3E, 0xFC60, 0xFC8D, 0xFCC1, 0xFCFD, 0xFD3D, 0xFD80,
        0xFDC7, 0xFE0F, 0xFE5A, 0xFEA5, 0xFEF1, 0xFF3E, 0xFF8A, 0xFFD6,
        0x001F, 0x0066, 0x00A8, 0x00E4, 0x011A, 0x0149, 0x016E, 0x0189,
        0x019B, 0x01A1, 0x019E, 0x0190, 0x0179, 0x015B, 0x0136, 0x010C,
        0x00DF, 0x00B0, 0x0082, 0x0056, 0x002D, 0x0009, 0xFFEB, 0xFFD3,
        0xFFC2, 0xFFB7, 0xFFB2, 0xFFB2, 0xFFB7, 0xFFC0, 0xFFCB, 0xFFD9,
        0xFFE6, 0xFFF3, 0xFFFF, 0x0009, 0x0011, 0x0016, 0x0018, 0x0018,
        0x0015, 0x0011, 0x000A, 0x0003, 0xFFFA, 0xFFF1, 0xFFE8, 0xFFE0,
        0xFFD7, 0xFFCF, 0xFFC8, 0xFFC1, 0xFFBB, 0xFFB5, 0xFFAF, 0xFFA9,
        0xFFA4, 0xFF9F, 0xFF9A, 0xFF96, 0xFF92, 0xFF90, 0xFF90, 0xFF91,
        0xFF94, 0xFF9A, 0xFFA2, 0xFFAD, 0xFFBB, 0xFFCC, 0xFFDF, 0xFFF3,
        0x000A, 0x0021, 0x0039, 0x0050, 0x0066, 0x007A, 0x008B, 0x0099,
        0x00A4, 0x00AB, 0x00AF, 0x00AE, 0x00A9, 0x00A1, 0x0095, 0x0087,
        0x0076, 0x0064, 0x0051, 0x003D, 0x002A, 0x0016, 0x0004, 0xFFF3,
        0xFFE3, 0xFFD5, 0xFFC9, 0xFFBE, 0xFFB5, 0xFFAD, 0xFFA7, 0xFFA2,
        0xFF9F, 0xFF9C, 0xFF9B, 0xFF9B, 0xFF9C, 0xFF9E, 0xFFA1, 0xFFA6,
        0xFFAC, 0xFFB3, 0xFFBB, 0xFFC5, 0xFFCF, 0xFFDA, 0xFFE6, 0xFFF2,
        0xFFFE, 0x000A, 0x0014, 0x001E, 0x0025, 0x002C, 0x0030, 0x0032,
        0x0033, 0x0031, 0x002E, 0x0029, 0x0023, 0x001D, 0x0016, 0x000E,
        0x0008, 0x0002, 0xFFFD, 0xFFF9, 0xFFF7, 0xFFF7, 0xFFF8, 0xFFFA,
        0xFFFE, 0x0003, 0x0009, 0x000F, 0x0016, 0x001C, 0x0022, 0x0028,
        0x002C, 0x002F, 0x0032, 0x0033, 0x0033, 0x0031, 0x002F, 0x002C,
        0x0028, 0x0024, 0x001F, 0x001A, 0x0014, 0x000E, 0x0007, 0x0001,
        0xFFFA, 0xFFF3, 0xFFEB, 0xFFE4, 0xFFDC, 0xFFD3, 0xFFCB, 0xFFC3,
        0xFFBB, 0xFFB4, 0xFFAD, 0xFFA8, 0xFFA4, 0xFFA1, 0xFFA0, 0xFFA1,
        0xFFA4, 0xFFAA, 0xFFB1, 0xFFBB, 0xFFC6, 0xFFD3, 0xFFE1, 0xFFF0
    },
    {
        0x0000, 0xFFF7, 0xFFEF, 0xFFE8, 0xFFE3, 0xFFE0, 0xFFE0, 0xFFE1,
        0xFFE3, 0xFFE7, 0xFFEB, 0xFFF0, 0xFFF4, 0xFFF8, 0xFFFC, 0xFFFF,
        0x0003, 0x0006, 0x0008, 0x000B, 0x000D, 0x000E, 0x000E, 0x000D,
        0x000C, 0x0009, 0x0006, 0x0003, 0x0001, 0xFFFF, 0xFFFE, 0xFFFF,
        0x0001, 0x0004, 0x0009, 0x000D, 0x0011, 0x0013, 0x0015, 0x0014,
        0x0012, 0x000D, 0x0008, 0x0001, 0xFFFB, 0xFFF4, 0xFFEE, 0xFFE9,
        0xFFE5, 0xFFE3, 0xFFE1, 0xFFE0, 0xFFE1, 0xFFE2, 0xFFE4, 0xFFE7,
        0xFFEC, 0xFFF2, 0xFFF9, 0x0001, 0x0009, 0x0012, 0x001A, 0x0021,
        0x0025, 0x0028, 0x0028, 0x0025, 0x0020, 0x0019, 0x0012, 0x000A,
        0x0003, 0xFFFC, 0xFFF8, 0xFFF5, 0xFFF4, 0xFFF4, 0xFFF5, 0xFFF6,
        0xFFF8, 0xFFF9, 0xFFFA, 0xFFFA, 0xFFFA, 0xFFFA, 0xFFFA, 0xFFFA,
        0xFFFA, 0xFFFA, 0xFFF9, 0xFFF8, 0xFFF6, 0xFFF3, 0xFFEF, 0xFFEC,
        0xFFE8, 0xFFE6, 0xFFE5, 0xFFE7, 0xFFEB, 0xFFF2, 0xFFFB, 0x0006,
        0x0012, 0x001E, 0x0029, 0x0031, 0x0037, 0x0039, 0x0038, 0x0033,
        0x002D, 0x0024, 0x001A, 0x0010, 0x0006, 0xFFFC, 0xFFF2, 0xFFE9,
        0xFFE1, 0xFFDA, 0xFFD3, 0xFFCE, 0xFFCB, 0xFFC9, 0xFFCB, 0xFFD0,
        0xFFD8, 0xFFE2, 0xFFED, 0xFFFA, 0x0006, 0x0010, 0x0018, 0x001C,
        0x001C, 0x0019, 0x0014, 0x000D, 0x0005, 0xFFFF, 0xFFFA, 0xFFF9,
        0xFFFA, 0xFFFF, 0x0006, 0x000E, 0x0016, 0x001F, 0x0026, 0x002B,
        0x002F, 0x0031, 0x0031, 0x002F, 0x002D, 0x0028, 0x0022, 0x0019,
        0x000D, 0xFFFE, 0xFFEB, 0xFFD6, 0xFFBF, 0xFFA7, 0xFF92, 0xFF82,
        0xFF78, 0xFF76, 0xFF7D, 0xFF8D, 0xFFA6, 0xFFC5, 0xFFE8, 0x000C,
        0x002E, 0x004C, 0x0064, 0x0074, 0x007E, 0x0081, 0x007F, 0x0079,
        0x0070, 0x0064, 0x0057, 0x0048, 0x0038, 0x0025, 0x0010, 0xFFFA,
        0xFFE4, 0xFFD0, 0xFFBF, 0xFFB5, 0xFFB3, 0xFFBA, 0xFFCA, 0xFFDF,
        0xFFF8, 0x0010, 0x0022, 0x0028, 0x0021, 0x0009, 0xFFE1, 0xFFAC,
        0xFF70, 0xFF33, 0xFEFC, 0xFED3, 0xFEBE, 0xFEC1, 0xFEDE, 0xFF13,
        0xFF5E, 0xFFB9, 0x0020, 0x008C, 0x00FB, 0x0169, 0x01D5, 0x023D,
        0x029F, 0x02F8, 0x0341, 0x0373, 0x037F, 0x0358, 0x02EF, 0x0238,
        0x012B, 0xFFCB, 0xFE22, 0xFC48, 0xFA60, 0xF897, 0xF71F, 0xF62A,
        0xF5E6, 0xF675, 0xF7E5, 0xFA31, 0xFD3A, 0x00CB, 0x049B, 0x0855,
        0x0B9D, 0x0E1E, 0x0F91, 0x0FC7, 0x0EAC, 0x0C4F, 0x08DE, 0x04A4,
        0x0000, 0xFB5C, 0xF722, 0xF3B1, 0xF154, 0xF039, 0xF06F, 0xF1E2,
        0xF463, 0xF7AB, 0xFB65, 0xFF35, 0x02C6, 0x05CF, 0x081B, 0x098B,
        0x0A1A, 0x09D6, 0x08E1, 0x0769, 0x05A0, 0x03B8, 0x01DE, 0x0035,
        0xFED5, 0xFDC8, 0xFD11, 0xFCA8, 0xFC81, 0xFC8D, 0xFCBF, 0xFD08,
        0xFD61, 0xFDC3, 0xFE2B, 0xFE97, 0xFF05, 0xFF74, 0xFFE0, 0x0047,
        0x00A2, 0x00ED, 0x0122, 0x013F, 0x0142, 0x012D, 0x0104, 0x00CD,
        0x0090, 0x0054, 0x001F, 0xFFF7, 0xFFDF, 0xFFD8, 0xFFDE, 0xFFF0,
        0x0008, 0x0021, 0x0036, 0x0046, 0x004D, 0x004B, 0x0041, 0x0030,
        0x001C, 0x0006, 0xFFF0, 0xFFDB, 0xFFC8, 0xFFB8, 0xFFA9, 0xFF9C,
        0xFF90, 0xFF87, 0xFF81, 0xFF7F, 0xFF82, 0xFF8C, 0xFF9C, 0xFFB4,
        0xFFD2, 0xFFF4, 0x0018, 0x003B, 0x005A, 0x0073, 0x0083, 0x008A,
        0x0088, 0x007E, 0x006E, 0x0059, 0x0041, 0x002A, 0x0015, 0x0002,
        0xFFF3, 0xFFE7, 0xFFDE, 0xFFD8, 0xFFD3, 0xFFD1, 0xFFCF, 0xFFCF,
        0xFFD1, 0xFFD5, 0xFFDA, 0xFFE1, 0xFFEA, 0xFFF2, 0xFFFA, 0x0001,
        0x0006, 0x0007, 0x0006, 0x0001, 0xFFFB, 0xFFF3, 0xFFEC, 0xFFE7,
        0xFFE4, 0xFFE4, 0xFFE8, 0xFFF0, 0xFFFA, 0x0006, 0x0013, 0x001E,
        0x0028, 0x0030, 0x0035, 0x0037, 0x0035, 0x0032, 0x002D, 0x0026,
        0x001F, 0x0017, 0x000E, 0x0004, 0xFFFA, 0xFFF0, 0xFFE6, 0xFFDC,
        0xFFD3, 0xFFCD, 0xFFC8, 0xFFC7, 0xFFC9, 0xFFCF, 0xFFD7, 0xFFE2,
        0xFFEE, 0xFFFA, 0x0005, 0x000E, 0x0015, 0x0019, 0x001B, 0x001A,
        0x0018, 0x0014, 0x0011, 0x000D, 0x000A, 0x0008, 0x0007, 0x0006,
        0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x0007,
        0x0008, 0x000A, 0x000B, 0x000C, 0x000C, 0x000B, 0x0008, 0x0004,
        0xFFFD, 0xFFF6, 0xFFEE, 0xFFE7, 0xFFE0, 0xFFDB, 0xFFD8, 0xFFD8,
        0xFFDB, 0xFFDF, 0xFFE6, 0xFFEE, 0xFFF7, 0xFFFF, 0x0007, 0x000E,
        0x0014, 0x0019, 0x001C, 0x001E, 0x001F, 0x0020, 0x001F, 0x001D,
        0x001B, 0x0017, 0x0012, 0x000C, 0x0005, 0xFFFF, 0xFFF8, 0xFFF3,
        0xFFEE, 0xFFEC, 0xFFEB, 0xFFED, 0xFFEF, 0xFFF3, 0xFFF7, 0xFFFC,
        0xFFFF, 0x0001, 0x0002, 0x0001, 0xFFFF, 0xFFFD, 0xFFFA, 0xFFF7,
        0xFFF4, 0xFFF3, 0xFFF2, 0xFFF2, 0xFFF3, 0xFFF5, 0xFFF8, 0xFFFA,
        0xFFFD, 0x0001, 0x0004, 0x0008, 0x000C, 0x0010, 0x0015, 0x0019,
        0x001D, 0x001F, 0x0020, 0x0020, 0x001D, 0x0018, 0x0011, 0x0009
    },
    {
        0x0000, 0x0005, 0x0009, 0x000B, 0x000B, 0x000A, 0x0007, 0x0005,
        0x0002, 0x0000, 0xFFFE, 0xFFFC, 0xFFFB, 0xFFFB, 0xFFFC, 0xFFFD,
        0xFFFF, 0x0000, 0x0001, 0xFFFF, 0xFFFD, 0xFFFB, 0xFFFA, 0xFFFA,
        0xFFFB, 0xFFFE, 0x0001, 0x0005, 0x0007, 0x0009, 0x000A, 0x000B,
        0x000A, 0x0008, 0x0004, 0x0000, 0xFFFA, 0xFFF6, 0xFFF4, 0xFFF4,
        0xFFF6, 0xFFF9, 0xFFFC, 0xFFFF, 0x0001, 0x0003, 0x0003, 0x0004,
        0x0004, 0x0004, 0x0003, 0x0001, 0x0000, 0x0000, 0x0001, 0x0003,
        0x0006, 0x0008, 0x0008, 0x0006, 0x0002, 0xFFFE, 0xFFF9, 0xFFF6,
        0xFFF4, 0xFFF4, 0xFFF4, 0xFFF6, 0xFFF9, 0xFFFD, 0x0002, 0x0007,
        0x000B, 0x000E, 0x000E, 0x000C, 0x0008, 0x0003, 0xFFFF, 0xFFFD,
        0xFFFC, 0xFFFC, 0xFFFD, 0xFFFE, 0xFFFE, 0xFFFE, 0xFFFE, 0xFFFE,
        0xFFFE, 0xFFFD, 0xFFFB, 0xFFF9, 0xFFF7, 0xFFF7, 0xFFF9, 0xFFFD,
        0x0004, 0x000A, 0x000F, 0x0012, 0x0011, 0x000F, 0x000B, 0x0006,
        0x0001, 0xFFFB, 0xFFF7, 0xFFF2, 0xFFEF, 0xFFEF, 0xFFF1, 0xFFF6,
        0xFFFC, 0x0002, 0x0006, 0x0007, 0x0006, 0x0003, 0x0001, 0x0000,
        0x0000, 0x0002, 0x0004, 0x0006, 0x0008, 0x0009, 0x000B, 0x000B,
        0x000B, 0x0008, 0x0002, 0xFFFA, 0xFFF1, 0xFFE9, 0xFFE5, 0xFFE5,
        0xFFEA, 0xFFF2, 0xFFFB, 0x0004, 0x000B, 0x0011, 0x0014, 0x0016,
        0x0016, 0x0013, 0x000D, 0x0006, 0xFFFE, 0xFFF7, 0xFFF5, 0xFFF6,
        0xFFFB, 0x0000, 0x0003, 0x0003, 0xFFFF, 0xFFF9, 0xFFF2, 0xFFED,
        0xFFEB, 0xFFEB, 0xFFEE, 0xFFF2, 0xFFF8, 0x0001, 0x000E, 0x001C,
        0x0029, 0x0031, 0x0031, 0x0028, 0x0017, 0x0003, 0xFFF1, 0xFFE2,
        0xFFDA, 0xFFD8, 0xFFD9, 0xFFDE, 0xFFE4, 0xFFED, 0xFFF8, 0x0003,
        0x000E, 0x0013, 0x0013, 0x000B, 0x0001, 0xFFF9, 0xFFF7, 0x0000,
        0x0010, 0x0024, 0x0034, 0x003B, 0x0038, 0x002C, 0x0018, 0x0001,
        0xFFE8, 0xFFCD, 0xFFB2, 0xFF99, 0xFF89, 0xFF89, 0xFF9F, 0xFFC9,
        0x0002, 0x003C, 0x006A, 0x0083, 0x0083, 0x0071, 0x0054, 0x0036,
        0x001C, 0x0006, 0xFFF1, 0xFFDE, 0xFFD1, 0xFFCE, 0xFFDB, 0xFFF5,
        0x000F, 0x0015, 0xFFF8, 0xFFB1, 0xFF4F, 0xFEEF, 0xFEB6, 0xFEC3,
        0xFF20, 0xFFC2, 0x008B, 0x0159, 0x0215, 0x02B0, 0x0321, 0x0355,
        0x0322, 0x024F, 0x00A9, 0xFE29, 0xFB1C, 0xF828, 0xF638, 0xF634,
        0xF8A9, 0xFD80, 0x03D8, 0x0A31, 0x0ECB, 0x1033, 0x0DC3, 0x07E4,
        0x0000, 0xF81C, 0xF23D, 0xEFCD, 0xF135, 0xF5CF, 0xFC28, 0x0280,
        0x0757, 0x09CC, 0x09C8, 0x07D8, 0x04E4, 0x01D7, 0xFF57, 0xFDB1,
        0xFCDE, 0xFCAB, 0xFCDF, 0xFD50, 0xFDEB, 0xFEA7, 0xFF75, 0x003E,
        0x00E0, 0x013D, 0x014A, 0x0111, 0x00B1, 0x004F, 0x0008, 0xFFEB,
        0xFFF1, 0x000B, 0x0025, 0x0032, 0x002F, 0x0022, 0x000F, 0xFFFA,
        0xFFE4, 0xFFCA, 0xFFAC, 0xFF8F, 0xFF7D, 0xFF7D, 0xFF96, 0xFFC4,
        0xFFFE, 0x0037, 0x0061, 0x0077, 0x0077, 0x0067, 0x004E, 0x0033,
        0x0018, 0xFFFF, 0xFFE8, 0xFFD4, 0xFFC8, 0xFFC5, 0xFFCC, 0xFFDC,
        0xFFF0, 0x0000, 0x0009, 0x0007, 0xFFFF, 0xFFF5, 0xFFED, 0xFFED,
        0xFFF2, 0xFFFD, 0x0008, 0x0013, 0x001C, 0x0022, 0x0027, 0x0028,
        0x0026, 0x001E, 0x000F, 0xFFFD, 0xFFE9, 0xFFD8, 0xFFCF, 0xFFCF,
        0xFFD7, 0xFFE4, 0xFFF2, 0xFFFF, 0x0008, 0x000E, 0x0012, 0x0015,
        0x0015, 0x0013, 0x000E, 0x0007, 0x0001, 0xFFFD, 0xFFFD, 0x0000,
        0x0005, 0x000A, 0x000B, 0x0009, 0x0002, 0xFFFA, 0xFFF3, 0xFFED,
        0xFFEA, 0xFFEA, 0xFFEC, 0xFFEF, 0xFFF5, 0xFFFC, 0x0005, 0x000E,
        0x0016, 0x001B, 0x001B, 0x0017, 0x000F, 0x0006, 0xFFFE, 0xFFF8,
        0xFFF5, 0xFFF5, 0xFFF5, 0xFFF7, 0xFFF8, 0xFFFA, 0xFFFC, 0xFFFE,
        0x0000, 0x0000, 0xFFFF, 0xFFFD, 0xFFFA, 0xFFF9, 0xFFFA, 0xFFFE,
        0x0004, 0x000A, 0x000F, 0x0011, 0x0011, 0x000E, 0x0009, 0x0005,
        0xFFFF, 0xFFFA, 0xFFF5, 0xFFF1, 0xFFEF, 0xFFEE, 0xFFF1, 0xFFF6,
        0xFFFC, 0x0003, 0x0007, 0x0009, 0x0009, 0x0007, 0x0005, 0x0003,
        0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0002, 0x0003, 0x0004,
        0x0004, 0x0003, 0x0001, 0xFFFD, 0xFFF8, 0xFFF4, 0xFFF2, 0xFFF2,
        0xFFF5, 0xFFF9, 0xFFFE, 0x0003, 0x0007, 0x000A, 0x000C, 0x000C,
        0x000C, 0x000A, 0x0007, 0x0002, 0xFFFE, 0xFFFA, 0xFFF8, 0xFFF8,
        0xFFFA, 0xFFFD, 0xFFFF, 0x0000, 0x0000, 0xFFFF, 0xFFFD, 0xFFFC,
        0xFFFC, 0xFFFC, 0xFFFD, 0xFFFD, 0xFFFF, 0x0001, 0x0004, 0x0007,
        0x000A, 0x000C, 0x000C, 0x000A, 0x0006, 0x0000, 0xFFFC, 0xFFF8,
        0xFFF6, 0xFFF5, 0xFFF6, 0xFFF7, 0xFFF9, 0xFFFB, 0xFFFF, 0x0002,
        0x0005, 0x0006, 0x0006, 0x0005, 0x0003, 0x0001, 0xFFFF, 0x0000,
        0x0001, 0x0003, 0x0004, 0x0005, 0x0005, 0x0004, 0x0002, 0x0000,
        0xFFFE, 0xFFFB, 0xFFF9, 0xFFF6, 0xFFF5, 0xFFF5, 0xFFF7, 0xFFFB
    }
};

static const int16_t lcwSawFactors[] = {
    1, 2, 3, 5, 8, 13, 22, 37
};

const LCWOscWaveSource gLcwOscSawSource = {
    8,
    &(lcwSawTables[0]),
    &(lcwSawFactors[0])
};
