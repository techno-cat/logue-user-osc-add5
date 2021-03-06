/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "LCWOscWaveSource.h"

static const LCWOscWaveTable lcwPulseTables[] = {
    {
        0x0000, 0x00D1, 0x01A1, 0x0272, 0x0342, 0x0413, 0x04E3, 0x05B3,
        0x0683, 0x0752, 0x0822, 0x08F1, 0x09BF, 0x0A8D, 0x0B5B, 0x0C29,
        0x0CF6, 0x0DC2, 0x0E8E, 0x0F59, 0x1024, 0x10EE, 0x11B7, 0x1280,
        0x1348, 0x1410, 0x14D6, 0x159C, 0x1661, 0x1725, 0x17E8, 0x18AA,
        0x196C, 0x1A2C, 0x1AEB, 0x1BA9, 0x1C67, 0x1D23, 0x1DDE, 0x1E98,
        0x1F50, 0x2008, 0x20BE, 0x2173, 0x2226, 0x22D9, 0x238A, 0x2439,
        0x24E8, 0x2594, 0x2640, 0x26EA, 0x2792, 0x2839, 0x28DE, 0x2982,
        0x2A24, 0x2AC4, 0x2B63, 0x2C00, 0x2C9C, 0x2D36, 0x2DCE, 0x2E64,
        0x2EF8, 0x2F8B, 0x301C, 0x30AB, 0x3138, 0x31C3, 0x324C, 0x32D4,
        0x3359, 0x33DD, 0x345E, 0x34DD, 0x355B, 0x35D6, 0x364F, 0x36C6,
        0x373B, 0x37AE, 0x381F, 0x388D, 0x38FA, 0x3964, 0x39CC, 0x3A32,
        0x3A95, 0x3AF6, 0x3B55, 0x3BB2, 0x3C0C, 0x3C64, 0x3CBA, 0x3D0E,
        0x3D5F, 0x3DAD, 0x3DFA, 0x3E44, 0x3E8B, 0x3ED0, 0x3F13, 0x3F53,
        0x3F91, 0x3FCC, 0x4005, 0x403C, 0x406F, 0x40A1, 0x40D0, 0x40FC,
        0x4126, 0x414E, 0x4173, 0x4195, 0x41B5, 0x41D2, 0x41ED, 0x4205,
        0x421B, 0x422E, 0x423F, 0x424D, 0x4259, 0x4262, 0x4268, 0x426C,
        0x426D, 0x426C, 0x4268, 0x4262, 0x4259, 0x424D, 0x423F, 0x422E,
        0x421B, 0x4205, 0x41ED, 0x41D2, 0x41B5, 0x4195, 0x4173, 0x414E,
        0x4126, 0x40FC, 0x40D0, 0x40A1, 0x406F, 0x403C, 0x4005, 0x3FCC,
        0x3F91, 0x3F53, 0x3F13, 0x3ED0, 0x3E8B, 0x3E44, 0x3DFA, 0x3DAD,
        0x3D5F, 0x3D0E, 0x3CBA, 0x3C64, 0x3C0C, 0x3BB2, 0x3B55, 0x3AF6,
        0x3A95, 0x3A32, 0x39CC, 0x3964, 0x38FA, 0x388D, 0x381F, 0x37AE,
        0x373B, 0x36C6, 0x364F, 0x35D6, 0x355B, 0x34DD, 0x345E, 0x33DD,
        0x3359, 0x32D4, 0x324C, 0x31C3, 0x3138, 0x30AB, 0x301C, 0x2F8B,
        0x2EF8, 0x2E64, 0x2DCE, 0x2D36, 0x2C9C, 0x2C00, 0x2B63, 0x2AC4,
        0x2A24, 0x2982, 0x28DE, 0x2839, 0x2792, 0x26EA, 0x2640, 0x2594,
        0x24E8, 0x2439, 0x238A, 0x22D9, 0x2226, 0x2173, 0x20BE, 0x2008,
        0x1F50, 0x1E98, 0x1DDE, 0x1D23, 0x1C67, 0x1BA9, 0x1AEB, 0x1A2C,
        0x196C, 0x18AA, 0x17E8, 0x1725, 0x1661, 0x159C, 0x14D6, 0x1410,
        0x1348, 0x1280, 0x11B7, 0x10EE, 0x1024, 0x0F59, 0x0E8E, 0x0DC2,
        0x0CF6, 0x0C29, 0x0B5B, 0x0A8D, 0x09BF, 0x08F1, 0x0822, 0x0752,
        0x0683, 0x05B3, 0x04E3, 0x0413, 0x0342, 0x0272, 0x01A1, 0x00D1,
        0x0000, 0xFF2F, 0xFE5F, 0xFD8E, 0xFCBE, 0xFBED, 0xFB1D, 0xFA4D,
        0xF97D, 0xF8AE, 0xF7DE, 0xF70F, 0xF641, 0xF573, 0xF4A5, 0xF3D7,
        0xF30A, 0xF23E, 0xF172, 0xF0A7, 0xEFDC, 0xEF12, 0xEE49, 0xED80,
        0xECB8, 0xEBF0, 0xEB2A, 0xEA64, 0xE99F, 0xE8DB, 0xE818, 0xE756,
        0xE694, 0xE5D4, 0xE515, 0xE457, 0xE399, 0xE2DD, 0xE222, 0xE168,
        0xE0B0, 0xDFF8, 0xDF42, 0xDE8D, 0xDDDA, 0xDD27, 0xDC76, 0xDBC7,
        0xDB18, 0xDA6C, 0xD9C0, 0xD916, 0xD86E, 0xD7C7, 0xD722, 0xD67E,
        0xD5DC, 0xD53C, 0xD49D, 0xD400, 0xD364, 0xD2CA, 0xD232, 0xD19C,
        0xD108, 0xD075, 0xCFE4, 0xCF55, 0xCEC8, 0xCE3D, 0xCDB4, 0xCD2C,
        0xCCA7, 0xCC23, 0xCBA2, 0xCB23, 0xCAA5, 0xCA2A, 0xC9B1, 0xC93A,
        0xC8C5, 0xC852, 0xC7E1, 0xC773, 0xC706, 0xC69C, 0xC634, 0xC5CE,
        0xC56B, 0xC50A, 0xC4AB, 0xC44E, 0xC3F4, 0xC39C, 0xC346, 0xC2F2,
        0xC2A1, 0xC253, 0xC206, 0xC1BC, 0xC175, 0xC130, 0xC0ED, 0xC0AD,
        0xC06F, 0xC034, 0xBFFB, 0xBFC4, 0xBF91, 0xBF5F, 0xBF30, 0xBF04,
        0xBEDA, 0xBEB2, 0xBE8D, 0xBE6B, 0xBE4B, 0xBE2E, 0xBE13, 0xBDFB,
        0xBDE5, 0xBDD2, 0xBDC1, 0xBDB3, 0xBDA7, 0xBD9E, 0xBD98, 0xBD94,
        0xBD93, 0xBD94, 0xBD98, 0xBD9E, 0xBDA7, 0xBDB3, 0xBDC1, 0xBDD2,
        0xBDE5, 0xBDFB, 0xBE13, 0xBE2E, 0xBE4B, 0xBE6B, 0xBE8D, 0xBEB2,
        0xBEDA, 0xBF04, 0xBF30, 0xBF5F, 0xBF91, 0xBFC4, 0xBFFB, 0xC034,
        0xC06F, 0xC0AD, 0xC0ED, 0xC130, 0xC175, 0xC1BC, 0xC206, 0xC253,
        0xC2A1, 0xC2F2, 0xC346, 0xC39C, 0xC3F4, 0xC44E, 0xC4AB, 0xC50A,
        0xC56B, 0xC5CE, 0xC634, 0xC69C, 0xC706, 0xC773, 0xC7E1, 0xC852,
        0xC8C5, 0xC93A, 0xC9B1, 0xCA2A, 0xCAA5, 0xCB23, 0xCBA2, 0xCC23,
        0xCCA7, 0xCD2C, 0xCDB4, 0xCE3D, 0xCEC8, 0xCF55, 0xCFE4, 0xD075,
        0xD108, 0xD19C, 0xD232, 0xD2CA, 0xD364, 0xD400, 0xD49D, 0xD53C,
        0xD5DC, 0xD67E, 0xD722, 0xD7C7, 0xD86E, 0xD916, 0xD9C0, 0xDA6C,
        0xDB18, 0xDBC7, 0xDC76, 0xDD27, 0xDDDA, 0xDE8D, 0xDF42, 0xDFF8,
        0xE0B0, 0xE168, 0xE222, 0xE2DD, 0xE399, 0xE457, 0xE515, 0xE5D4,
        0xE694, 0xE756, 0xE818, 0xE8DB, 0xE99F, 0xEA64, 0xEB2A, 0xEBF0,
        0xECB8, 0xED80, 0xEE49, 0xEF12, 0xEFDC, 0xF0A7, 0xF172, 0xF23E,
        0xF30A, 0xF3D7, 0xF4A5, 0xF573, 0xF641, 0xF70F, 0xF7DE, 0xF8AE,
        0xF97D, 0xFA4D, 0xFB1D, 0xFBED, 0xFCBE, 0xFD8E, 0xFE5F, 0xFF2F
    },
    {
        0x0000, 0x00D1, 0x01A1, 0x0271, 0x0340, 0x040E, 0x04DA, 0x05A5,
        0x066D, 0x0734, 0x07F8, 0x08B9, 0x0978, 0x0A33, 0x0AEA, 0x0B9E,
        0x0C4D, 0x0CF9, 0x0D9F, 0x0E41, 0x0EDF, 0x0F77, 0x1009, 0x1096,
        0x111E, 0x119F, 0x121A, 0x128F, 0x12FE, 0x1366, 0x13C7, 0x1421,
        0x1475, 0x14C1, 0x1506, 0x1544, 0x157A, 0x15A9, 0x15D1, 0x15F1,
        0x1609, 0x161A, 0x1623, 0x1624, 0x161E, 0x160F, 0x15FA, 0x15DC,
        0x15B7, 0x158B, 0x1557, 0x151C, 0x14D9, 0x148F, 0x143E, 0x13E6,
        0x1387, 0x1321, 0x12B5, 0x1242, 0x11C9, 0x114A, 0x10C4, 0x1039,
        0x0FA8, 0x0F12, 0x0E76, 0x0DD6, 0x0D31, 0x0C87, 0x0BD9, 0x0B26,
        0x0A70, 0x09B6, 0x08F9, 0x0839, 0x0776, 0x06B0, 0x05E8, 0x051E,
        0x0452, 0x0384, 0x02B6, 0x01E6, 0x0116, 0x0046, 0xFF75, 0xFEA4,
        0xFDD4, 0xFD05, 0xFC37, 0xFB6A, 0xFA9F, 0xF9D5, 0xF90E, 0xF849,
        0xF787, 0xF6C8, 0xF60B, 0xF553, 0xF49E, 0xF3ED, 0xF340, 0xF298,
        0xF1F4, 0xF155, 0xF0BB, 0xF027, 0xEF98, 0xEF0F, 0xEE8B, 0xEE0E,
        0xED97, 0xED26, 0xECBC, 0xEC59, 0xEBFC, 0xEBA6, 0xEB57, 0xEB10,
        0xEAD0, 0xEA97, 0xEA65, 0xEA3B, 0xEA19, 0xE9FE, 0xE9EB, 0xE9DF,
        0xE9DC, 0xE9DF, 0xE9EB, 0xE9FE, 0xEA19, 0xEA3B, 0xEA65, 0xEA97,
        0xEAD0, 0xEB10, 0xEB57, 0xEBA6, 0xEBFC, 0xEC59, 0xECBC, 0xED26,
        0xED97, 0xEE0E, 0xEE8B, 0xEF0F, 0xEF98, 0xF027, 0xF0BB, 0xF155,
        0xF1F4, 0xF298, 0xF340, 0xF3ED, 0xF49E, 0xF553, 0xF60B, 0xF6C8,
        0xF787, 0xF849, 0xF90E, 0xF9D5, 0xFA9F, 0xFB6A, 0xFC37, 0xFD05,
        0xFDD4, 0xFEA4, 0xFF75, 0x0046, 0x0116, 0x01E6, 0x02B6, 0x0384,
        0x0452, 0x051E, 0x05E8, 0x06B0, 0x0776, 0x0839, 0x08F9, 0x09B6,
        0x0A70, 0x0B26, 0x0BD9, 0x0C87, 0x0D31, 0x0DD6, 0x0E76, 0x0F12,
        0x0FA8, 0x1039, 0x10C4, 0x114A, 0x11C9, 0x1242, 0x12B5, 0x1321,
        0x1387, 0x13E6, 0x143E, 0x148F, 0x14D9, 0x151C, 0x1557, 0x158B,
        0x15B7, 0x15DC, 0x15FA, 0x160F, 0x161E, 0x1624, 0x1623, 0x161A,
        0x1609, 0x15F1, 0x15D1, 0x15A9, 0x157A, 0x1544, 0x1506, 0x14C1,
        0x1475, 0x1421, 0x13C7, 0x1366, 0x12FE, 0x128F, 0x121A, 0x119F,
        0x111E, 0x1096, 0x1009, 0x0F77, 0x0EDF, 0x0E41, 0x0D9F, 0x0CF9,
        0x0C4D, 0x0B9E, 0x0AEA, 0x0A33, 0x0978, 0x08B9, 0x07F8, 0x0734,
        0x066D, 0x05A5, 0x04DA, 0x040E, 0x0340, 0x0271, 0x01A1, 0x00D1,
        0x0000, 0xFF2F, 0xFE5F, 0xFD8F, 0xFCC0, 0xFBF2, 0xFB26, 0xFA5B,
        0xF993, 0xF8CC, 0xF808, 0xF747, 0xF688, 0xF5CD, 0xF516, 0xF462,
        0xF3B3, 0xF307, 0xF261, 0xF1BF, 0xF121, 0xF089, 0xEFF7, 0xEF6A,
        0xEEE2, 0xEE61, 0xEDE6, 0xED71, 0xED02, 0xEC9A, 0xEC39, 0xEBDF,
        0xEB8B, 0xEB3F, 0xEAFA, 0xEABC, 0xEA86, 0xEA57, 0xEA2F, 0xEA0F,
        0xE9F7, 0xE9E6, 0xE9DD, 0xE9DC, 0xE9E2, 0xE9F1, 0xEA06, 0xEA24,
        0xEA49, 0xEA75, 0xEAA9, 0xEAE4, 0xEB27, 0xEB71, 0xEBC2, 0xEC1A,
        0xEC79, 0xECDF, 0xED4B, 0xEDBE, 0xEE37, 0xEEB6, 0xEF3C, 0xEFC7,
        0xF058, 0xF0EE, 0xF18A, 0xF22A, 0xF2CF, 0xF379, 0xF427, 0xF4DA,
        0xF590, 0xF64A, 0xF707, 0xF7C7, 0xF88A, 0xF950, 0xFA18, 0xFAE2,
        0xFBAE, 0xFC7C, 0xFD4A, 0xFE1A, 0xFEEA, 0xFFBA, 0x008B, 0x015C,
        0x022C, 0x02FB, 0x03C9, 0x0496, 0x0561, 0x062B, 0x06F2, 0x07B7,
        0x0879, 0x0938, 0x09F5, 0x0AAD, 0x0B62, 0x0C13, 0x0CC0, 0x0D68,
        0x0E0C, 0x0EAB, 0x0F45, 0x0FD9, 0x1068, 0x10F1, 0x1175, 0x11F2,
        0x1269, 0x12DA, 0x1344, 0x13A7, 0x1404, 0x145A, 0x14A9, 0x14F0,
        0x1530, 0x1569, 0x159B, 0x15C5, 0x15E7, 0x1602, 0x1615, 0x1621,
        0x1624, 0x1621, 0x1615, 0x1602, 0x15E7, 0x15C5, 0x159B, 0x1569,
        0x1530, 0x14F0, 0x14A9, 0x145A, 0x1404, 0x13A7, 0x1344, 0x12DA,
        0x1269, 0x11F2, 0x1175, 0x10F1, 0x1068, 0x0FD9, 0x0F45, 0x0EAB,
        0x0E0C, 0x0D68, 0x0CC0, 0x0C13, 0x0B62, 0x0AAD, 0x09F5, 0x0938,
        0x0879, 0x07B7, 0x06F2, 0x062B, 0x0561, 0x0496, 0x03C9, 0x02FB,
        0x022C, 0x015C, 0x008B, 0xFFBA, 0xFEEA, 0xFE1A, 0xFD4A, 0xFC7C,
        0xFBAE, 0xFAE2, 0xFA18, 0xF950, 0xF88A, 0xF7C7, 0xF707, 0xF64A,
        0xF590, 0xF4DA, 0xF427, 0xF379, 0xF2CF, 0xF22A, 0xF18A, 0xF0EE,
        0xF058, 0xEFC7, 0xEF3C, 0xEEB6, 0xEE37, 0xEDBE, 0xED4B, 0xECDF,
        0xEC79, 0xEC1A, 0xEBC2, 0xEB71, 0xEB27, 0xEAE4, 0xEAA9, 0xEA75,
        0xEA49, 0xEA24, 0xEA06, 0xE9F1, 0xE9E2, 0xE9DC, 0xE9DD, 0xE9E6,
        0xE9F7, 0xEA0F, 0xEA2F, 0xEA57, 0xEA86, 0xEABC, 0xEAFA, 0xEB3F,
        0xEB8B, 0xEBDF, 0xEC39, 0xEC9A, 0xED02, 0xED71, 0xEDE6, 0xEE61,
        0xEEE2, 0xEF6A, 0xEFF7, 0xF089, 0xF121, 0xF1BF, 0xF261, 0xF307,
        0xF3B3, 0xF462, 0xF516, 0xF5CD, 0xF688, 0xF747, 0xF808, 0xF8CC,
        0xF993, 0xFA5B, 0xFB26, 0xFBF2, 0xFCC0, 0xFD8F, 0xFE5F, 0xFF2F
    },
    {
        0x0000, 0x0139, 0x0270, 0x03A4, 0x04D4, 0x05FD, 0x071F, 0x0837,
        0x0946, 0x0A48, 0x0B3E, 0x0C26, 0x0CFE, 0x0DC6, 0x0E7D, 0x0F21,
        0x0FB3, 0x1032, 0x109C, 0x10F2, 0x1134, 0x1160, 0x1178, 0x117A,
        0x1168, 0x1141, 0x1106, 0x10B6, 0x1054, 0x0FDF, 0x0F58, 0x0EBF,
        0x0E17, 0x0D5F, 0x0C99, 0x0BC6, 0x0AE7, 0x09FE, 0x090A, 0x080F,
        0x070D, 0x0605, 0x04F9, 0x03EB, 0x02DB, 0x01CA, 0x00BB, 0xFFAE,
        0xFEA6, 0xFDA2, 0xFCA4, 0xFBAD, 0xFABF, 0xF9DA, 0xF8FF, 0xF830,
        0xF76C, 0xF6B5, 0xF60B, 0xF56F, 0xF4E1, 0xF463, 0xF3F3, 0xF392,
        0xF340, 0xF2FE, 0xF2CB, 0xF2A8, 0xF294, 0xF28E, 0xF297, 0xF2AE,
        0xF2D2, 0xF304, 0xF342, 0xF38D, 0xF3E2, 0xF442, 0xF4AC, 0xF520,
        0xF59B, 0xF61E, 0xF6A8, 0xF738, 0xF7CD, 0xF867, 0xF904, 0xF9A3,
        0xFA45, 0xFAE9, 0xFB8C, 0xFC30, 0xFCD3, 0xFD75, 0xFE15, 0xFEB2,
        0xFF4D, 0xFFE4, 0x0077, 0x0106, 0x0191, 0x0217, 0x0298, 0x0314,
        0x038A, 0x03FB, 0x0466, 0x04CC, 0x052C, 0x0586, 0x05DB, 0x062B,
        0x0675, 0x06B9, 0x06F8, 0x0732, 0x0768, 0x0798, 0x07C3, 0x07EA,
        0x080C, 0x082B, 0x0844, 0x085A, 0x086B, 0x0879, 0x0883, 0x0888,
        0x088A, 0x0888, 0x0883, 0x0879, 0x086B, 0x085A, 0x0844, 0x082B,
        0x080C, 0x07EA, 0x07C3, 0x0798, 0x0768, 0x0732, 0x06F8, 0x06B9,
        0x0675, 0x062B, 0x05DB, 0x0586, 0x052C, 0x04CC, 0x0466, 0x03FB,
        0x038A, 0x0314, 0x0298, 0x0217, 0x0191, 0x0106, 0x0077, 0xFFE4,
        0xFF4D, 0xFEB2, 0xFE15, 0xFD75, 0xFCD3, 0xFC30, 0xFB8C, 0xFAE9,
        0xFA45, 0xF9A3, 0xF904, 0xF867, 0xF7CD, 0xF738, 0xF6A8, 0xF61E,
        0xF59B, 0xF520, 0xF4AC, 0xF442, 0xF3E2, 0xF38D, 0xF342, 0xF304,
        0xF2D2, 0xF2AE, 0xF297, 0xF28E, 0xF294, 0xF2A8, 0xF2CB, 0xF2FE,
        0xF340, 0xF392, 0xF3F3, 0xF463, 0xF4E1, 0xF56F, 0xF60B, 0xF6B5,
        0xF76C, 0xF830, 0xF8FF, 0xF9DA, 0xFABF, 0xFBAD, 0xFCA4, 0xFDA2,
        0xFEA6, 0xFFAE, 0x00BB, 0x01CA, 0x02DB, 0x03EB, 0x04F9, 0x0605,
        0x070D, 0x080F, 0x090A, 0x09FE, 0x0AE7, 0x0BC6, 0x0C99, 0x0D5F,
        0x0E17, 0x0EBF, 0x0F58, 0x0FDF, 0x1054, 0x10B6, 0x1106, 0x1141,
        0x1168, 0x117A, 0x1178, 0x1160, 0x1134, 0x10F2, 0x109C, 0x1032,
        0x0FB3, 0x0F21, 0x0E7D, 0x0DC6, 0x0CFE, 0x0C26, 0x0B3E, 0x0A48,
        0x0946, 0x0837, 0x071F, 0x05FD, 0x04D4, 0x03A4, 0x0270, 0x0139,
        0x0000, 0xFEC7, 0xFD90, 0xFC5C, 0xFB2C, 0xFA03, 0xF8E1, 0xF7C9,
        0xF6BA, 0xF5B8, 0xF4C2, 0xF3DA, 0xF302, 0xF23A, 0xF183, 0xF0DF,
        0xF04D, 0xEFCE, 0xEF64, 0xEF0E, 0xEECC, 0xEEA0, 0xEE88, 0xEE86,
        0xEE98, 0xEEBF, 0xEEFA, 0xEF4A, 0xEFAC, 0xF021, 0xF0A8, 0xF141,
        0xF1E9, 0xF2A1, 0xF367, 0xF43A, 0xF519, 0xF602, 0xF6F6, 0xF7F1,
        0xF8F3, 0xF9FB, 0xFB07, 0xFC15, 0xFD25, 0xFE36, 0xFF45, 0x0052,
        0x015A, 0x025E, 0x035C, 0x0453, 0x0541, 0x0626, 0x0701, 0x07D0,
        0x0894, 0x094B, 0x09F5, 0x0A91, 0x0B1F, 0x0B9D, 0x0C0D, 0x0C6E,
        0x0CC0, 0x0D02, 0x0D35, 0x0D58, 0x0D6C, 0x0D72, 0x0D69, 0x0D52,
        0x0D2E, 0x0CFC, 0x0CBE, 0x0C73, 0x0C1E, 0x0BBE, 0x0B54, 0x0AE0,
        0x0A65, 0x09E2, 0x0958, 0x08C8, 0x0833, 0x0799, 0x06FC, 0x065D,
        0x05BB, 0x0517, 0x0474, 0x03D0, 0x032D, 0x028B, 0x01EB, 0x014E,
        0x00B3, 0x001C, 0xFF89, 0xFEFA, 0xFE6F, 0xFDE9, 0xFD68, 0xFCEC,
        0xFC76, 0xFC05, 0xFB9A, 0xFB34, 0xFAD4, 0xFA7A, 0xFA25, 0xF9D5,
        0xF98B, 0xF947, 0xF908, 0xF8CE, 0xF898, 0xF868, 0xF83D, 0xF816,
        0xF7F4, 0xF7D5, 0xF7BC, 0xF7A6, 0xF795, 0xF787, 0xF77D, 0xF778,
        0xF776, 0xF778, 0xF77D, 0xF787, 0xF795, 0xF7A6, 0xF7BC, 0xF7D5,
        0xF7F4, 0xF816, 0xF83D, 0xF868, 0xF898, 0xF8CE, 0xF908, 0xF947,
        0xF98B, 0xF9D5, 0xFA25, 0xFA7A, 0xFAD4, 0xFB34, 0xFB9A, 0xFC05,
        0xFC76, 0xFCEC, 0xFD68, 0xFDE9, 0xFE6F, 0xFEFA, 0xFF89, 0x001C,
        0x00B3, 0x014E, 0x01EB, 0x028B, 0x032D, 0x03D0, 0x0474, 0x0517,
        0x05BB, 0x065D, 0x06FC, 0x0799, 0x0833, 0x08C8, 0x0958, 0x09E2,
        0x0A65, 0x0AE0, 0x0B54, 0x0BBE, 0x0C1E, 0x0C73, 0x0CBE, 0x0CFC,
        0x0D2E, 0x0D52, 0x0D69, 0x0D72, 0x0D6C, 0x0D58, 0x0D35, 0x0D02,
        0x0CC0, 0x0C6E, 0x0C0D, 0x0B9D, 0x0B1F, 0x0A91, 0x09F5, 0x094B,
        0x0894, 0x07D0, 0x0701, 0x0626, 0x0541, 0x0453, 0x035C, 0x025E,
        0x015A, 0x0052, 0xFF45, 0xFE36, 0xFD25, 0xFC15, 0xFB07, 0xF9FB,
        0xF8F3, 0xF7F1, 0xF6F6, 0xF602, 0xF519, 0xF43A, 0xF367, 0xF2A1,
        0xF1E9, 0xF141, 0xF0A8, 0xF021, 0xEFAC, 0xEF4A, 0xEEFA, 0xEEBF,
        0xEE98, 0xEE86, 0xEE88, 0xEEA0, 0xEECC, 0xEF0E, 0xEF64, 0xEFCE,
        0xF04D, 0xF0DF, 0xF183, 0xF23A, 0xF302, 0xF3DA, 0xF4C2, 0xF5B8,
        0xF6BA, 0xF7C9, 0xF8E1, 0xFA03, 0xFB2C, 0xFC5C, 0xFD90, 0xFEC7
    },
    {
        0x0000, 0x0208, 0x0409, 0x05FB, 0x07D7, 0x0995, 0x0B30, 0x0CA2,
        0x0DE5, 0x0EF6, 0x0FD1, 0x1073, 0x10DB, 0x1107, 0x10F8, 0x10B0,
        0x1030, 0x0F7A, 0x0E93, 0x0D7F, 0x0C42, 0x0AE3, 0x0967, 0x07D4,
        0x0630, 0x0483, 0x02D2, 0x0124, 0xFF80, 0xFDE9, 0xFC67, 0xFAFE,
        0xF9B3, 0xF888, 0xF781, 0xF6A1, 0xF5E8, 0xF558, 0xF4F2, 0xF4B4,
        0xF49D, 0xF4AD, 0xF4E1, 0xF536, 0xF5A9, 0xF637, 0xF6DD, 0xF796,
        0xF860, 0xF935, 0xFA13, 0xFAF5, 0xFBD9, 0xFCBA, 0xFD96, 0xFE6B,
        0xFF36, 0xFFF4, 0x00A5, 0x0148, 0x01DA, 0x025D, 0x02CE, 0x0330,
        0x0382, 0x03C4, 0x03F7, 0x041D, 0x0437, 0x0445, 0x0448, 0x0443,
        0x0436, 0x0421, 0x0407, 0x03E9, 0x03C6, 0x03A0, 0x0377, 0x034C,
        0x031F, 0x02F0, 0x02C1, 0x0290, 0x025E, 0x022A, 0x01F6, 0x01C1,
        0x018A, 0x0152, 0x011A, 0x00E1, 0x00A7, 0x006D, 0x0033, 0xFFFA,
        0xFFC2, 0xFF8B, 0xFF57, 0xFF25, 0xFEF7, 0xFECC, 0xFEA7, 0xFE86,
        0xFE6B, 0xFE56, 0xFE47, 0xFE3F, 0xFE3D, 0xFE42, 0xFE4D, 0xFE5E,
        0xFE74, 0xFE90, 0xFEB0, 0xFED4, 0xFEFA, 0xFF23, 0xFF4C, 0xFF75,
        0xFF9C, 0xFFC2, 0xFFE5, 0x0004, 0x001F, 0x0034, 0x0044, 0x004D,
        0x0050, 0x004D, 0x0044, 0x0034, 0x001F, 0x0004, 0xFFE5, 0xFFC2,
        0xFF9C, 0xFF75, 0xFF4C, 0xFF23, 0xFEFA, 0xFED4, 0xFEB0, 0xFE90,
        0xFE74, 0xFE5E, 0xFE4D, 0xFE42, 0xFE3D, 0xFE3F, 0xFE47, 0xFE56,
        0xFE6B, 0xFE86, 0xFEA7, 0xFECC, 0xFEF7, 0xFF25, 0xFF57, 0xFF8B,
        0xFFC2, 0xFFFA, 0x0033, 0x006D, 0x00A7, 0x00E1, 0x011A, 0x0152,
        0x018A, 0x01C1, 0x01F6, 0x022A, 0x025E, 0x0290, 0x02C1, 0x02F0,
        0x031F, 0x034C, 0x0377, 0x03A0, 0x03C6, 0x03E9, 0x0407, 0x0421,
        0x0436, 0x0443, 0x0448, 0x0445, 0x0437, 0x041D, 0x03F7, 0x03C4,
        0x0382, 0x0330, 0x02CE, 0x025D, 0x01DA, 0x0148, 0x00A5, 0xFFF4,
        0xFF36, 0xFE6B, 0xFD96, 0xFCBA, 0xFBD9, 0xFAF5, 0xFA13, 0xF935,
        0xF860, 0xF796, 0xF6DD, 0xF637, 0xF5A9, 0xF536, 0xF4E1, 0xF4AD,
        0xF49D, 0xF4B4, 0xF4F2, 0xF558, 0xF5E8, 0xF6A1, 0xF781, 0xF888,
        0xF9B3, 0xFAFE, 0xFC67, 0xFDE9, 0xFF80, 0x0124, 0x02D2, 0x0483,
        0x0630, 0x07D4, 0x0967, 0x0AE3, 0x0C42, 0x0D7F, 0x0E93, 0x0F7A,
        0x1030, 0x10B0, 0x10F8, 0x1107, 0x10DB, 0x1073, 0x0FD1, 0x0EF6,
        0x0DE5, 0x0CA2, 0x0B30, 0x0995, 0x07D7, 0x05FB, 0x0409, 0x0208,
        0x0000, 0xFDF8, 0xFBF7, 0xFA05, 0xF829, 0xF66B, 0xF4D0, 0xF35E,
        0xF21B, 0xF10A, 0xF02F, 0xEF8D, 0xEF25, 0xEEF9, 0xEF08, 0xEF50,
        0xEFD0, 0xF086, 0xF16D, 0xF281, 0xF3BE, 0xF51D, 0xF699, 0xF82C,
        0xF9D0, 0xFB7D, 0xFD2E, 0xFEDC, 0x0080, 0x0217, 0x0399, 0x0502,
        0x064D, 0x0778, 0x087F, 0x095F, 0x0A18, 0x0AA8, 0x0B0E, 0x0B4C,
        0x0B63, 0x0B53, 0x0B1F, 0x0ACA, 0x0A57, 0x09C9, 0x0923, 0x086A,
        0x07A0, 0x06CB, 0x05ED, 0x050B, 0x0427, 0x0346, 0x026A, 0x0195,
        0x00CA, 0x000C, 0xFF5B, 0xFEB8, 0xFE26, 0xFDA3, 0xFD32, 0xFCD0,
        0xFC7E, 0xFC3C, 0xFC09, 0xFBE3, 0xFBC9, 0xFBBB, 0xFBB8, 0xFBBD,
        0xFBCA, 0xFBDF, 0xFBF9, 0xFC17, 0xFC3A, 0xFC60, 0xFC89, 0xFCB4,
        0xFCE1, 0xFD10, 0xFD3F, 0xFD70, 0xFDA2, 0xFDD6, 0xFE0A, 0xFE3F,
        0xFE76, 0xFEAE, 0xFEE6, 0xFF1F, 0xFF59, 0xFF93, 0xFFCD, 0x0006,
        0x003E, 0x0075, 0x00A9, 0x00DB, 0x0109, 0x0134, 0x0159, 0x017A,
        0x0195, 0x01AA, 0x01B9, 0x01C1, 0x01C3, 0x01BE, 0x01B3, 0x01A2,
        0x018C, 0x0170, 0x0150, 0x012C, 0x0106, 0x00DD, 0x00B4, 0x008B,
        0x0064, 0x003E, 0x001B, 0xFFFC, 0xFFE1, 0xFFCC, 0xFFBC, 0xFFB3,
        0xFFB0, 0xFFB3, 0xFFBC, 0xFFCC, 0xFFE1, 0xFFFC, 0x001B, 0x003E,
        0x0064, 0x008B, 0x00B4, 0x00DD, 0x0106, 0x012C, 0x0150, 0x0170,
        0x018C, 0x01A2, 0x01B3, 0x01BE, 0x01C3, 0x01C1, 0x01B9, 0x01AA,
        0x0195, 0x017A, 0x0159, 0x0134, 0x0109, 0x00DB, 0x00A9, 0x0075,
        0x003E, 0x0006, 0xFFCD, 0xFF93, 0xFF59, 0xFF1F, 0xFEE6, 0xFEAE,
        0xFE76, 0xFE3F, 0xFE0A, 0xFDD6, 0xFDA2, 0xFD70, 0xFD3F, 0xFD10,
        0xFCE1, 0xFCB4, 0xFC89, 0xFC60, 0xFC3A, 0xFC17, 0xFBF9, 0xFBDF,
        0xFBCA, 0xFBBD, 0xFBB8, 0xFBBB, 0xFBC9, 0xFBE3, 0xFC09, 0xFC3C,
        0xFC7E, 0xFCD0, 0xFD32, 0xFDA3, 0xFE26, 0xFEB8, 0xFF5B, 0x000C,
        0x00CA, 0x0195, 0x026A, 0x0346, 0x0427, 0x050B, 0x05ED, 0x06CB,
        0x07A0, 0x086A, 0x0923, 0x09C9, 0x0A57, 0x0ACA, 0x0B1F, 0x0B53,
        0x0B63, 0x0B4C, 0x0B0E, 0x0AA8, 0x0A18, 0x095F, 0x087F, 0x0778,
        0x064D, 0x0502, 0x0399, 0x0217, 0x0080, 0xFEDC, 0xFD2E, 0xFB7D,
        0xF9D0, 0xF82C, 0xF699, 0xF51D, 0xF3BE, 0xF281, 0xF16D, 0xF086,
        0xEFD0, 0xEF50, 0xEF08, 0xEEF9, 0xEF25, 0xEF8D, 0xF02F, 0xF10A,
        0xF21B, 0xF35E, 0xF4D0, 0xF66B, 0xF829, 0xFA05, 0xFBF7, 0xFDF8
    },
    {
        0x0000, 0x02D6, 0x0590, 0x0814, 0x0A4B, 0x0C1E, 0x0D7E, 0x0E5F,
        0x0EBB, 0x0E8F, 0x0DE0, 0x0CB9, 0x0B26, 0x093A, 0x070A, 0x04AD,
        0x023B, 0xFFCC, 0xFD78, 0xFB54, 0xF971, 0xF7E0, 0xF6AA, 0xF5D7,
        0xF56A, 0xF560, 0xF5B3, 0xF65B, 0xF74D, 0xF879, 0xF9D1, 0xFB45,
        0xFCC4, 0xFE40, 0xFFAB, 0x00F8, 0x0220, 0x031A, 0x03E1, 0x0474,
        0x04D4, 0x0502, 0x0503, 0x04DC, 0x0494, 0x0431, 0x03BC, 0x033A,
        0x02B2, 0x022A, 0x01A6, 0x012A, 0x00B9, 0x0053, 0xFFFA, 0xFFAD,
        0xFF6D, 0xFF37, 0xFF0C, 0xFEE9, 0xFECF, 0xFEBB, 0xFEAD, 0xFEA5,
        0xFEA2, 0xFEA5, 0xFEAD, 0xFEBB, 0xFECE, 0xFEE7, 0xFF06, 0xFF28,
        0xFF4F, 0xFF78, 0xFFA2, 0xFFCC, 0xFFF4, 0x0018, 0x0037, 0x004F,
        0x005F, 0x0067, 0x0066, 0x005D, 0x004D, 0x0037, 0x001D, 0xFFFF,
        0xFFE2, 0xFFC6, 0xFFAE, 0xFF9B, 0xFF90, 0xFF8C, 0xFF92, 0xFFA1,
        0xFFB8, 0xFFD7, 0xFFFC, 0x0026, 0x0052, 0x007F, 0x00A9, 0x00D0,
        0x00F1, 0x010B, 0x011C, 0x0124, 0x0122, 0x0117, 0x0103, 0x00E7,
        0x00C5, 0x009D, 0x0072, 0x0045, 0x0017, 0xFFEB, 0xFFC0, 0xFF98,
        0xFF74, 0xFF54, 0xFF39, 0xFF22, 0xFF10, 0xFF02, 0xFEF8, 0xFEF2,
        0xFEF0, 0xFEF2, 0xFEF8, 0xFF02, 0xFF10, 0xFF22, 0xFF39, 0xFF54,
        0xFF74, 0xFF98, 0xFFC0, 0xFFEB, 0x0017, 0x0045, 0x0072, 0x009D,
        0x00C5, 0x00E7, 0x0103, 0x0117, 0x0122, 0x0124, 0x011C, 0x010B,
        0x00F1, 0x00D0, 0x00A9, 0x007F, 0x0052, 0x0026, 0xFFFC, 0xFFD7,
        0xFFB8, 0xFFA1, 0xFF92, 0xFF8C, 0xFF90, 0xFF9B, 0xFFAE, 0xFFC6,
        0xFFE2, 0xFFFF, 0x001D, 0x0037, 0x004D, 0x005D, 0x0066, 0x0067,
        0x005F, 0x004F, 0x0037, 0x0018, 0xFFF4, 0xFFCC, 0xFFA2, 0xFF78,
        0xFF4F, 0xFF28, 0xFF06, 0xFEE7, 0xFECE, 0xFEBB, 0xFEAD, 0xFEA5,
        0xFEA2, 0xFEA5, 0xFEAD, 0xFEBB, 0xFECF, 0xFEE9, 0xFF0C, 0xFF37,
        0xFF6D, 0xFFAD, 0xFFFA, 0x0053, 0x00B9, 0x012A, 0x01A6, 0x022A,
        0x02B2, 0x033A, 0x03BC, 0x0431, 0x0494, 0x04DC, 0x0503, 0x0502,
        0x04D4, 0x0474, 0x03E1, 0x031A, 0x0220, 0x00F8, 0xFFAB, 0xFE40,
        0xFCC4, 0xFB45, 0xF9D1, 0xF879, 0xF74D, 0xF65B, 0xF5B3, 0xF560,
        0xF56A, 0xF5D7, 0xF6AA, 0xF7E0, 0xF971, 0xFB54, 0xFD78, 0xFFCC,
        0x023B, 0x04AD, 0x070A, 0x093A, 0x0B26, 0x0CB9, 0x0DE0, 0x0E8F,
        0x0EBB, 0x0E5F, 0x0D7E, 0x0C1E, 0x0A4B, 0x0814, 0x0590, 0x02D6,
        0x0000, 0xFD2A, 0xFA70, 0xF7EC, 0xF5B5, 0xF3E2, 0xF282, 0xF1A1,
        0xF145, 0xF171, 0xF220, 0xF347, 0xF4DA, 0xF6C6, 0xF8F6, 0xFB53,
        0xFDC5, 0x0034, 0x0288, 0x04AC, 0x068F, 0x0820, 0x0956, 0x0A29,
        0x0A96, 0x0AA0, 0x0A4D, 0x09A5, 0x08B3, 0x0787, 0x062F, 0x04BB,
        0x033C, 0x01C0, 0x0055, 0xFF08, 0xFDE0, 0xFCE6, 0xFC1F, 0xFB8C,
        0xFB2C, 0xFAFE, 0xFAFD, 0xFB24, 0xFB6C, 0xFBCF, 0xFC44, 0xFCC6,
        0xFD4E, 0xFDD6, 0xFE5A, 0xFED6, 0xFF47, 0xFFAD, 0x0006, 0x0053,
        0x0093, 0x00C9, 0x00F4, 0x0117, 0x0131, 0x0145, 0x0153, 0x015B,
        0x015E, 0x015B, 0x0153, 0x0145, 0x0132, 0x0119, 0x00FA, 0x00D8,
        0x00B1, 0x0088, 0x005E, 0x0034, 0x000C, 0xFFE8, 0xFFC9, 0xFFB1,
        0xFFA1, 0xFF99, 0xFF9A, 0xFFA3, 0xFFB3, 0xFFC9, 0xFFE3, 0x0001,
        0x001E, 0x003A, 0x0052, 0x0065, 0x0070, 0x0074, 0x006E, 0x005F,
        0x0048, 0x0029, 0x0004, 0xFFDA, 0xFFAE, 0xFF81, 0xFF57, 0xFF30,
        0xFF0F, 0xFEF5, 0xFEE4, 0xFEDC, 0xFEDE, 0xFEE9, 0xFEFD, 0xFF19,
        0xFF3B, 0xFF63, 0xFF8E, 0xFFBB, 0xFFE9, 0x0015, 0x0040, 0x0068,
        0x008C, 0x00AC, 0x00C7, 0x00DE, 0x00F0, 0x00FE, 0x0108, 0x010E,
        0x0110, 0x010E, 0x0108, 0x00FE, 0x00F0, 0x00DE, 0x00C7, 0x00AC,
        0x008C, 0x0068, 0x0040, 0x0015, 0xFFE9, 0xFFBB, 0xFF8E, 0xFF63,
        0xFF3B, 0xFF19, 0xFEFD, 0xFEE9, 0xFEDE, 0xFEDC, 0xFEE4, 0xFEF5,
        0xFF0F, 0xFF30, 0xFF57, 0xFF81, 0xFFAE, 0xFFDA, 0x0004, 0x0029,
        0x0048, 0x005F, 0x006E, 0x0074, 0x0070, 0x0065, 0x0052, 0x003A,
        0x001E, 0x0001, 0xFFE3, 0xFFC9, 0xFFB3, 0xFFA3, 0xFF9A, 0xFF99,
        0xFFA1, 0xFFB1, 0xFFC9, 0xFFE8, 0x000C, 0x0034, 0x005E, 0x0088,
        0x00B1, 0x00D8, 0x00FA, 0x0119, 0x0132, 0x0145, 0x0153, 0x015B,
        0x015E, 0x015B, 0x0153, 0x0145, 0x0131, 0x0117, 0x00F4, 0x00C9,
        0x0093, 0x0053, 0x0006, 0xFFAD, 0xFF47, 0xFED6, 0xFE5A, 0xFDD6,
        0xFD4E, 0xFCC6, 0xFC44, 0xFBCF, 0xFB6C, 0xFB24, 0xFAFD, 0xFAFE,
        0xFB2C, 0xFB8C, 0xFC1F, 0xFCE6, 0xFDE0, 0xFF08, 0x0055, 0x01C0,
        0x033C, 0x04BB, 0x062F, 0x0787, 0x08B3, 0x09A5, 0x0A4D, 0x0AA0,
        0x0A96, 0x0A29, 0x0956, 0x0820, 0x068F, 0x04AC, 0x0288, 0x0034,
        0xFDC5, 0xFB53, 0xF8F6, 0xF6C6, 0xF4DA, 0xF347, 0xF220, 0xF171,
        0xF145, 0xF1A1, 0xF282, 0xF3E2, 0xF5B5, 0xF7EC, 0xFA70, 0xFD2A
    },
    {
        0x0000, 0x0404, 0x07AA, 0x0A9F, 0x0CA1, 0x0D86, 0x0D40, 0x0BDC,
        0x0984, 0x0677, 0x0301, 0xFF77, 0xFC28, 0xF95E, 0xF74D, 0xF618,
        0xF5CA, 0xF656, 0xF79F, 0xF976, 0xFBA5, 0xFDF3, 0x002B, 0x021E,
        0x03AA, 0x04BD, 0x0550, 0x0569, 0x051A, 0x0479, 0x03A2, 0x02AF,
        0x01B8, 0x00D1, 0x0005, 0xFF5E, 0xFEDE, 0xFE83, 0xFE49, 0xFE2C,
        0xFE27, 0xFE34, 0xFE52, 0xFE7D, 0xFEB4, 0xFEF5, 0xFF3F, 0xFF8E,
        0xFFDE, 0x002C, 0x0070, 0x00A8, 0x00CD, 0x00DF, 0x00DC, 0x00C6,
        0x00A0, 0x0071, 0x003E, 0x000E, 0xFFE7, 0xFFCD, 0xFFC3, 0xFFC8,
        0xFFDB, 0xFFF8, 0x0019, 0x003A, 0x0056, 0x0067, 0x006C, 0x0064,
        0x004F, 0x0031, 0x000B, 0xFFE4, 0xFFBD, 0xFF9C, 0xFF82, 0xFF71,
        0xFF69, 0xFF6A, 0xFF72, 0xFF82, 0xFF96, 0xFFAD, 0xFFC8, 0xFFE4,
        0x0002, 0x0021, 0x0040, 0x005E, 0x0079, 0x0091, 0x00A2, 0x00AB,
        0x00AA, 0x009E, 0x0088, 0x0068, 0x0040, 0x0013, 0xFFE5, 0xFFBB,
        0xFF96, 0xFF7B, 0xFF6C, 0xFF68, 0xFF70, 0xFF82, 0xFF9C, 0xFFB9,
        0xFFD8, 0xFFF4, 0x000C, 0x001E, 0x0029, 0x002E, 0x002E, 0x002A,
        0x0024, 0x001E, 0x0018, 0x0013, 0x0010, 0x000F, 0x000E, 0x000E,
        0x000E, 0x000E, 0x000E, 0x000F, 0x0010, 0x0013, 0x0018, 0x001E,
        0x0024, 0x002A, 0x002E, 0x002E, 0x0029, 0x001E, 0x000C, 0xFFF4,
        0xFFD8, 0xFFB9, 0xFF9C, 0xFF82, 0xFF70, 0xFF68, 0xFF6C, 0xFF7B,
        0xFF96, 0xFFBB, 0xFFE5, 0x0013, 0x0040, 0x0068, 0x0088, 0x009E,
        0x00AA, 0x00AB, 0x00A2, 0x0091, 0x0079, 0x005E, 0x0040, 0x0021,
        0x0002, 0xFFE4, 0xFFC8, 0xFFAD, 0xFF96, 0xFF82, 0xFF72, 0xFF6A,
        0xFF69, 0xFF71, 0xFF82, 0xFF9C, 0xFFBD, 0xFFE4, 0x000B, 0x0031,
        0x004F, 0x0064, 0x006C, 0x0067, 0x0056, 0x003A, 0x0019, 0xFFF8,
        0xFFDB, 0xFFC8, 0xFFC3, 0xFFCD, 0xFFE7, 0x000E, 0x003E, 0x0071,
        0x00A0, 0x00C6, 0x00DC, 0x00DF, 0x00CD, 0x00A8, 0x0070, 0x002C,
        0xFFDE, 0xFF8E, 0xFF3F, 0xFEF5, 0xFEB4, 0xFE7D, 0xFE52, 0xFE34,
        0xFE27, 0xFE2C, 0xFE49, 0xFE83, 0xFEDE, 0xFF5E, 0x0005, 0x00D1,
        0x01B8, 0x02AF, 0x03A2, 0x0479, 0x051A, 0x0569, 0x0550, 0x04BD,
        0x03AA, 0x021E, 0x002B, 0xFDF3, 0xFBA5, 0xF976, 0xF79F, 0xF656,
        0xF5CA, 0xF618, 0xF74D, 0xF95E, 0xFC28, 0xFF77, 0x0301, 0x0677,
        0x0984, 0x0BDC, 0x0D40, 0x0D86, 0x0CA1, 0x0A9F, 0x07AA, 0x0404,
        0x0000, 0xFBFC, 0xF856, 0xF561, 0xF35F, 0xF27A, 0xF2C0, 0xF424,
        0xF67C, 0xF989, 0xFCFF, 0x0089, 0x03D8, 0x06A2, 0x08B3, 0x09E8,
        0x0A36, 0x09AA, 0x0861, 0x068A, 0x045B, 0x020D, 0xFFD5, 0xFDE2,
        0xFC56, 0xFB43, 0xFAB0, 0xFA97, 0xFAE6, 0xFB87, 0xFC5E, 0xFD51,
        0xFE48, 0xFF2F, 0xFFFB, 0x00A2, 0x0122, 0x017D, 0x01B7, 0x01D4,
        0x01D9, 0x01CC, 0x01AE, 0x0183, 0x014C, 0x010B, 0x00C1, 0x0072,
        0x0022, 0xFFD4, 0xFF90, 0xFF58, 0xFF33, 0xFF21, 0xFF24, 0xFF3A,
        0xFF60, 0xFF8F, 0xFFC2, 0xFFF2, 0x0019, 0x0033, 0x003D, 0x0038,
        0x0025, 0x0008, 0xFFE7, 0xFFC6, 0xFFAA, 0xFF99, 0xFF94, 0xFF9C,
        0xFFB1, 0xFFCF, 0xFFF5, 0x001C, 0x0043, 0x0064, 0x007E, 0x008F,
        0x0097, 0x0096, 0x008E, 0x007E, 0x006A, 0x0053, 0x0038, 0x001C,
        0xFFFE, 0xFFDF, 0xFFC0, 0xFFA2, 0xFF87, 0xFF6F, 0xFF5E, 0xFF55,
        0xFF56, 0xFF62, 0xFF78, 0xFF98, 0xFFC0, 0xFFED, 0x001B, 0x0045,
        0x006A, 0x0085, 0x0094, 0x0098, 0x0090, 0x007E, 0x0064, 0x0047,
        0x0028, 0x000C, 0xFFF4, 0xFFE2, 0xFFD7, 0xFFD2, 0xFFD2, 0xFFD6,
        0xFFDC, 0xFFE2, 0xFFE8, 0xFFED, 0xFFF0, 0xFFF1, 0xFFF2, 0xFFF2,
        0xFFF2, 0xFFF2, 0xFFF2, 0xFFF1, 0xFFF0, 0xFFED, 0xFFE8, 0xFFE2,
        0xFFDC, 0xFFD6, 0xFFD2, 0xFFD2, 0xFFD7, 0xFFE2, 0xFFF4, 0x000C,
        0x0028, 0x0047, 0x0064, 0x007E, 0x0090, 0x0098, 0x0094, 0x0085,
        0x006A, 0x0045, 0x001B, 0xFFED, 0xFFC0, 0xFF98, 0xFF78, 0xFF62,
        0xFF56, 0xFF55, 0xFF5E, 0xFF6F, 0xFF87, 0xFFA2, 0xFFC0, 0xFFDF,
        0xFFFE, 0x001C, 0x0038, 0x0053, 0x006A, 0x007E, 0x008E, 0x0096,
        0x0097, 0x008F, 0x007E, 0x0064, 0x0043, 0x001C, 0xFFF5, 0xFFCF,
        0xFFB1, 0xFF9C, 0xFF94, 0xFF99, 0xFFAA, 0xFFC6, 0xFFE7, 0x0008,
        0x0025, 0x0038, 0x003D, 0x0033, 0x0019, 0xFFF2, 0xFFC2, 0xFF8F,
        0xFF60, 0xFF3A, 0xFF24, 0xFF21, 0xFF33, 0xFF58, 0xFF90, 0xFFD4,
        0x0022, 0x0072, 0x00C1, 0x010B, 0x014C, 0x0183, 0x01AE, 0x01CC,
        0x01D9, 0x01D4, 0x01B7, 0x017D, 0x0122, 0x00A2, 0xFFFB, 0xFF2F,
        0xFE48, 0xFD51, 0xFC5E, 0xFB87, 0xFAE6, 0xFA97, 0xFAB0, 0xFB43,
        0xFC56, 0xFDE2, 0xFFD5, 0x020D, 0x045B, 0x068A, 0x0861, 0x09AA,
        0x0A36, 0x09E8, 0x08B3, 0x06A2, 0x03D8, 0x0089, 0xFCFF, 0xF989,
        0xF67C, 0xF424, 0xF2C0, 0xF27A, 0xF35F, 0xF561, 0xF856, 0xFBFC
    },
    {
        0x0000, 0x05E7, 0x0A95, 0x0D19, 0x0CFF, 0x0A68, 0x05F8, 0x00B3,
        0xFBB5, 0xF7F6, 0xF614, 0xF63B, 0xF823, 0xFB32, 0xFEA7, 0x01CA,
        0x0411, 0x053B, 0x054C, 0x0480, 0x032F, 0x01B5, 0x005B, 0xFF4F,
        0xFE9E, 0xFE43, 0xFE2A, 0xFE43, 0xFE7D, 0xFED1, 0xFF37, 0xFFA9,
        0x001A, 0x007B, 0x00BE, 0x00D8, 0x00C7, 0x0092, 0x004C, 0x0007,
        0xFFD6, 0xFFC4, 0xFFD2, 0xFFF7, 0x0024, 0x0048, 0x0057, 0x004B,
        0x0029, 0xFFF9, 0xFFC7, 0xFF9F, 0xFF88, 0xFF83, 0xFF90, 0xFFA9,
        0xFFC9, 0xFFEC, 0x0010, 0x0033, 0x0053, 0x006C, 0x007C, 0x007D,
        0x006D, 0x004C, 0x001F, 0xFFED, 0xFFBF, 0xFF9E, 0xFF8F, 0xFF95,
        0xFFAC, 0xFFCD, 0xFFF0, 0x000D, 0x0020, 0x0027, 0x0025, 0x001E,
        0x0015, 0x000E, 0x0009, 0x0008, 0x0007, 0x0007, 0x0007, 0x0008,
        0x0009, 0x000C, 0x000F, 0x0011, 0x000F, 0x0007, 0xFFFA, 0xFFE7,
        0xFFD4, 0xFFC5, 0xFFBE, 0xFFC3, 0xFFD4, 0xFFEE, 0x000C, 0x0029,
        0x003E, 0x0048, 0x0047, 0x003C, 0x002A, 0x0016, 0x0002, 0xFFF1,
        0xFFE3, 0xFFD8, 0xFFD2, 0xFFD0, 0xFFD2, 0xFFDA, 0xFFE7, 0xFFF7,
        0x0008, 0x0017, 0x001F, 0x0020, 0x001A, 0x000E, 0x0002, 0xFFF8,
        0xFFF4, 0xFFF8, 0x0002, 0x000E, 0x001A, 0x0020, 0x001F, 0x0017,
        0x0008, 0xFFF7, 0xFFE7, 0xFFDA, 0xFFD2, 0xFFD0, 0xFFD2, 0xFFD8,
        0xFFE3, 0xFFF1, 0x0002, 0x0016, 0x002A, 0x003C, 0x0047, 0x0048,
        0x003E, 0x0029, 0x000C, 0xFFEE, 0xFFD4, 0xFFC3, 0xFFBE, 0xFFC5,
        0xFFD4, 0xFFE7, 0xFFFA, 0x0007, 0x000F, 0x0011, 0x000F, 0x000C,
        0x0009, 0x0008, 0x0007, 0x0007, 0x0007, 0x0008, 0x0009, 0x000E,
        0x0015, 0x001E, 0x0025, 0x0027, 0x0020, 0x000D, 0xFFF0, 0xFFCD,
        0xFFAC, 0xFF95, 0xFF8F, 0xFF9E, 0xFFBF, 0xFFED, 0x001F, 0x004C,
        0x006D, 0x007D, 0x007C, 0x006C, 0x0053, 0x0033, 0x0010, 0xFFEC,
        0xFFC9, 0xFFA9, 0xFF90, 0xFF83, 0xFF88, 0xFF9F, 0xFFC7, 0xFFF9,
        0x0029, 0x004B, 0x0057, 0x0048, 0x0024, 0xFFF7, 0xFFD2, 0xFFC4,
        0xFFD6, 0x0007, 0x004C, 0x0092, 0x00C7, 0x00D8, 0x00BE, 0x007B,
        0x001A, 0xFFA9, 0xFF37, 0xFED1, 0xFE7D, 0xFE43, 0xFE2A, 0xFE43,
        0xFE9E, 0xFF4F, 0x005B, 0x01B5, 0x032F, 0x0480, 0x054C, 0x053B,
        0x0411, 0x01CA, 0xFEA7, 0xFB32, 0xF823, 0xF63B, 0xF614, 0xF7F6,
        0xFBB5, 0x00B3, 0x05F8, 0x0A68, 0x0CFF, 0x0D19, 0x0A95, 0x05E7,
        0x0000, 0xFA19, 0xF56B, 0xF2E7, 0xF301, 0xF598, 0xFA08, 0xFF4D,
        0x044B, 0x080A, 0x09EC, 0x09C5, 0x07DD, 0x04CE, 0x0159, 0xFE36,
        0xFBEF, 0xFAC5, 0xFAB4, 0xFB80, 0xFCD1, 0xFE4B, 0xFFA5, 0x00B1,
        0x0162, 0x01BD, 0x01D6, 0x01BD, 0x0183, 0x012F, 0x00C9, 0x0057,
        0xFFE6, 0xFF85, 0xFF42, 0xFF28, 0xFF39, 0xFF6E, 0xFFB4, 0xFFF9,
        0x002A, 0x003C, 0x002E, 0x0009, 0xFFDC, 0xFFB8, 0xFFA9, 0xFFB5,
        0xFFD7, 0x0007, 0x0039, 0x0061, 0x0078, 0x007D, 0x0070, 0x0057,
        0x0037, 0x0014, 0xFFF0, 0xFFCD, 0xFFAD, 0xFF94, 0xFF84, 0xFF83,
        0xFF93, 0xFFB4, 0xFFE1, 0x0013, 0x0041, 0x0062, 0x0071, 0x006B,
        0x0054, 0x0033, 0x0010, 0xFFF3, 0xFFE0, 0xFFD9, 0xFFDB, 0xFFE2,
        0xFFEB, 0xFFF2, 0xFFF7, 0xFFF8, 0xFFF9, 0xFFF9, 0xFFF9, 0xFFF8,
        0xFFF7, 0xFFF4, 0xFFF1, 0xFFEF, 0xFFF1, 0xFFF9, 0x0006, 0x0019,
        0x002C, 0x003B, 0x0042, 0x003D, 0x002C, 0x0012, 0xFFF4, 0xFFD7,
        0xFFC2, 0xFFB8, 0xFFB9, 0xFFC4, 0xFFD6, 0xFFEA, 0xFFFE, 0x000F,
        0x001D, 0x0028, 0x002E, 0x0030, 0x002E, 0x0026, 0x0019, 0x0009,
        0xFFF8, 0xFFE9, 0xFFE1, 0xFFE0, 0xFFE6, 0xFFF2, 0xFFFE, 0x0008,
        0x000C, 0x0008, 0xFFFE, 0xFFF2, 0xFFE6, 0xFFE0, 0xFFE1, 0xFFE9,
        0xFFF8, 0x0009, 0x0019, 0x0026, 0x002E, 0x0030, 0x002E, 0x0028,
        0x001D, 0x000F, 0xFFFE, 0xFFEA, 0xFFD6, 0xFFC4, 0xFFB9, 0xFFB8,
        0xFFC2, 0xFFD7, 0xFFF4, 0x0012, 0x002C, 0x003D, 0x0042, 0x003B,
        0x002C, 0x0019, 0x0006, 0xFFF9, 0xFFF1, 0xFFEF, 0xFFF1, 0xFFF4,
        0xFFF7, 0xFFF8, 0xFFF9, 0xFFF9, 0xFFF9, 0xFFF8, 0xFFF7, 0xFFF2,
        0xFFEB, 0xFFE2, 0xFFDB, 0xFFD9, 0xFFE0, 0xFFF3, 0x0010, 0x0033,
        0x0054, 0x006B, 0x0071, 0x0062, 0x0041, 0x0013, 0xFFE1, 0xFFB4,
        0xFF93, 0xFF83, 0xFF84, 0xFF94, 0xFFAD, 0xFFCD, 0xFFF0, 0x0014,
        0x0037, 0x0057, 0x0070, 0x007D, 0x0078, 0x0061, 0x0039, 0x0007,
        0xFFD7, 0xFFB5, 0xFFA9, 0xFFB8, 0xFFDC, 0x0009, 0x002E, 0x003C,
        0x002A, 0xFFF9, 0xFFB4, 0xFF6E, 0xFF39, 0xFF28, 0xFF42, 0xFF85,
        0xFFE6, 0x0057, 0x00C9, 0x012F, 0x0183, 0x01BD, 0x01D6, 0x01BD,
        0x0162, 0x00B1, 0xFFA5, 0xFE4B, 0xFCD1, 0xFB80, 0xFAB4, 0xFAC5,
        0xFBEF, 0xFE36, 0x0159, 0x04CE, 0x07DD, 0x09C5, 0x09EC, 0x080A,
        0x044B, 0xFF4D, 0xFA08, 0xF598, 0xF301, 0xF2E7, 0xF56B, 0xFA19
    },
    {
        0x0000, 0x0845, 0x0CC8, 0x0BA5, 0x05B5, 0xFDF8, 0xF7F9, 0xF60D,
        0xF85F, 0xFD34, 0x020E, 0x04FB, 0x0557, 0x03C0, 0x016F, 0xFF78,
        0xFE5E, 0xFE17, 0xFE56, 0xFED6, 0xFF6E, 0x000A, 0x008A, 0x00CB,
        0x00B8, 0x0063, 0xFFFF, 0xFFC5, 0xFFD0, 0x0009, 0x0042, 0x004C,
        0x0020, 0xFFD5, 0xFF97, 0xFF83, 0xFF9D, 0xFFD2, 0x000D, 0x003D,
        0x005C, 0x0068, 0x005D, 0x0039, 0x0002, 0xFFC8, 0xFFA2, 0xFFA0,
        0xFFC1, 0xFFF3, 0x001D, 0x002E, 0x0027, 0x0013, 0x0003, 0xFFFE,
        0x0002, 0x0008, 0x000C, 0x000A, 0x0007, 0x0001, 0xFFF8, 0xFFEB,
        0xFFDD, 0xFFD7, 0xFFE0, 0xFFF8, 0x0018, 0x0031, 0x0038, 0x002A,
        0x000F, 0xFFF4, 0xFFE2, 0xFFDC, 0xFFE1, 0xFFEB, 0xFFF6, 0x0002,
        0x000C, 0x0012, 0x0012, 0x000B, 0x0001, 0xFFFA, 0xFFFB, 0x0004,
        0x000D, 0x0010, 0x0007, 0xFFF8, 0xFFE9, 0xFFE2, 0xFFE7, 0xFFF4,
        0x0003, 0x0010, 0x0019, 0x001C, 0x0019, 0x0010, 0x0002, 0xFFF1,
        0xFFE5, 0xFFE3, 0xFFEB, 0xFFFA, 0x0008, 0x000F, 0x000D, 0x0005,
        0xFFFF, 0xFFFD, 0x0001, 0x0006, 0x0009, 0x0009, 0x0005, 0x0001,
        0xFFFB, 0xFFF4, 0xFFED, 0xFFEA, 0xFFEE, 0xFFFA, 0x000B, 0x001B,
        0x0021, 0x001B, 0x000B, 0xFFFA, 0xFFEE, 0xFFEA, 0xFFED, 0xFFF4,
        0xFFFB, 0x0001, 0x0005, 0x0009, 0x0009, 0x0006, 0x0001, 0xFFFD,
        0xFFFF, 0x0005, 0x000D, 0x000F, 0x0008, 0xFFFA, 0xFFEB, 0xFFE3,
        0xFFE5, 0xFFF1, 0x0002, 0x0010, 0x0019, 0x001C, 0x0019, 0x0010,
        0x0003, 0xFFF4, 0xFFE7, 0xFFE2, 0xFFE9, 0xFFF8, 0x0007, 0x0010,
        0x000D, 0x0004, 0xFFFB, 0xFFFA, 0x0001, 0x000B, 0x0012, 0x0012,
        0x000C, 0x0002, 0xFFF6, 0xFFEB, 0xFFE1, 0xFFDC, 0xFFE2, 0xFFF4,
        0x000F, 0x002A, 0x0038, 0x0031, 0x0018, 0xFFF8, 0xFFE0, 0xFFD7,
        0xFFDD, 0xFFEB, 0xFFF8, 0x0001, 0x0007, 0x000A, 0x000C, 0x0008,
        0x0002, 0xFFFE, 0x0003, 0x0013, 0x0027, 0x002E, 0x001D, 0xFFF3,
        0xFFC1, 0xFFA0, 0xFFA2, 0xFFC8, 0x0002, 0x0039, 0x005D, 0x0068,
        0x005C, 0x003D, 0x000D, 0xFFD2, 0xFF9D, 0xFF83, 0xFF97, 0xFFD5,
        0x0020, 0x004C, 0x0042, 0x0009, 0xFFD0, 0xFFC5, 0xFFFF, 0x0063,
        0x00B8, 0x00CB, 0x008A, 0x000A, 0xFF6E, 0xFED6, 0xFE56, 0xFE17,
        0xFE5E, 0xFF78, 0x016F, 0x03C0, 0x0557, 0x04FB, 0x020E, 0xFD34,
        0xF85F, 0xF60D, 0xF7F9, 0xFDF8, 0x05B5, 0x0BA5, 0x0CC8, 0x0845,
        0x0000, 0xF7BB, 0xF338, 0xF45B, 0xFA4B, 0x0208, 0x0807, 0x09F3,
        0x07A1, 0x02CC, 0xFDF2, 0xFB05, 0xFAA9, 0xFC40, 0xFE91, 0x0088,
        0x01A2, 0x01E9, 0x01AA, 0x012A, 0x0092, 0xFFF6, 0xFF76, 0xFF35,
        0xFF48, 0xFF9D, 0x0001, 0x003B, 0x0030, 0xFFF7, 0xFFBE, 0xFFB4,
        0xFFE0, 0x002B, 0x0069, 0x007D, 0x0063, 0x002E, 0xFFF3, 0xFFC3,
        0xFFA4, 0xFF98, 0xFFA3, 0xFFC7, 0xFFFE, 0x0038, 0x005E, 0x0060,
        0x003F, 0x000D, 0xFFE3, 0xFFD2, 0xFFD9, 0xFFED, 0xFFFD, 0x0002,
        0xFFFE, 0xFFF8, 0xFFF4, 0xFFF6, 0xFFF9, 0xFFFF, 0x0008, 0x0015,
        0x0023, 0x0029, 0x0020, 0x0008, 0xFFE8, 0xFFCF, 0xFFC8, 0xFFD6,
        0xFFF1, 0x000C, 0x001E, 0x0024, 0x001F, 0x0015, 0x000A, 0xFFFE,
        0xFFF4, 0xFFEE, 0xFFEE, 0xFFF5, 0xFFFF, 0x0006, 0x0005, 0xFFFC,
        0xFFF3, 0xFFF0, 0xFFF9, 0x0008, 0x0017, 0x001E, 0x0019, 0x000C,
        0xFFFD, 0xFFF0, 0xFFE7, 0xFFE4, 0xFFE7, 0xFFF0, 0xFFFE, 0x000F,
        0x001B, 0x001D, 0x0015, 0x0006, 0xFFF8, 0xFFF1, 0xFFF3, 0xFFFB,
        0x0001, 0x0003, 0xFFFF, 0xFFFA, 0xFFF7, 0xFFF7, 0xFFFB, 0xFFFF,
        0x0005, 0x000C, 0x0013, 0x0016, 0x0012, 0x0006, 0xFFF5, 0xFFE5,
        0xFFDF, 0xFFE5, 0xFFF5, 0x0006, 0x0012, 0x0016, 0x0013, 0x000C,
        0x0005, 0xFFFF, 0xFFFB, 0xFFF7, 0xFFF7, 0xFFFA, 0xFFFF, 0x0003,
        0x0001, 0xFFFB, 0xFFF3, 0xFFF1, 0xFFF8, 0x0006, 0x0015, 0x001D,
        0x001B, 0x000F, 0xFFFE, 0xFFF0, 0xFFE7, 0xFFE4, 0xFFE7, 0xFFF0,
        0xFFFD, 0x000C, 0x0019, 0x001E, 0x0017, 0x0008, 0xFFF9, 0xFFF0,
        0xFFF3, 0xFFFC, 0x0005, 0x0006, 0xFFFF, 0xFFF5, 0xFFEE, 0xFFEE,
        0xFFF4, 0xFFFE, 0x000A, 0x0015, 0x001F, 0x0024, 0x001E, 0x000C,
        0xFFF1, 0xFFD6, 0xFFC8, 0xFFCF, 0xFFE8, 0x0008, 0x0020, 0x0029,
        0x0023, 0x0015, 0x0008, 0xFFFF, 0xFFF9, 0xFFF6, 0xFFF4, 0xFFF8,
        0xFFFE, 0x0002, 0xFFFD, 0xFFED, 0xFFD9, 0xFFD2, 0xFFE3, 0x000D,
        0x003F, 0x0060, 0x005E, 0x0038, 0xFFFE, 0xFFC7, 0xFFA3, 0xFF98,
        0xFFA4, 0xFFC3, 0xFFF3, 0x002E, 0x0063, 0x007D, 0x0069, 0x002B,
        0xFFE0, 0xFFB4, 0xFFBE, 0xFFF7, 0x0030, 0x003B, 0x0001, 0xFF9D,
        0xFF48, 0xFF35, 0xFF76, 0xFFF6, 0x0092, 0x012A, 0x01AA, 0x01E9,
        0x01A2, 0x0088, 0xFE91, 0xFC40, 0xFAA9, 0xFB05, 0xFDF2, 0x02CC,
        0x07A1, 0x09F3, 0x0807, 0x0208, 0xFA4B, 0xF45B, 0xF338, 0xF7BB
    }
};

static const int16_t lcwPulseFactors[] = {
    1, 3, 5, 9, 15, 23, 35, 53
};

const LCWOscWaveSource gLcwOscPulseSource = {
    8,
    &(lcwPulseTables[0]),
    &(lcwPulseFactors[0])
};

