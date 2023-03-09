/* Generated by Edge Impulse
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _EI_CLASSIFIER_TFLITE_TRAINED_H_
#define _EI_CLASSIFIER_TFLITE_TRAINED_H_

#define EI_CLASSIFIER_TFLITE_ARENA_SIZE 2598

#if defined __GNUC__
#define ALIGN(X) __attribute__((aligned(X)))
#elif defined _MSC_VER
#define ALIGN(X) __declspec(align(X))
#elif defined __TASKING__
#define ALIGN(X) __align(X)
#else
#define ALIGN(X)
#endif
ALIGN(16)
const unsigned char trained_tflite[] = {
    0x20, 0x00, 0x00, 0x00, 0x54, 0x46, 0x4c, 0x33, 0x00, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x20, 0x00, 0x1c, 0x00, 0x18, 0x00, 0x14, 0x00, 0x10, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x04, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x1c, 0x00, 0x00, 0x00, 0x8c, 0x00, 0x00, 0x00, 0xbc, 0x00, 0x00, 0x00,
    0x20, 0x06, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x70, 0x0c, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0a, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00,
    0x3c, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x73, 0x65, 0x72, 0x76,
    0x69, 0x6e, 0x67, 0x5f, 0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xc4, 0xff, 0xff, 0xff,
    0x0a, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00,
    0x6f, 0x75, 0x74, 0x70, 0x75, 0x74, 0x5f, 0x30, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xc6, 0xf9, 0xff, 0xff,
    0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00,
    0x08, 0x00, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x6e, 0x5f,
    0x72, 0x75, 0x6e, 0x74, 0x69, 0x6d, 0x65, 0x5f, 0x76, 0x65, 0x72, 0x73,
    0x69, 0x6f, 0x6e, 0x00, 0x0d, 0x00, 0x00, 0x00, 0x60, 0x05, 0x00, 0x00,
    0x58, 0x05, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0xa0, 0x01, 0x00, 0x00,
    0xc8, 0x00, 0x00, 0x00, 0x90, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x44, 0x00, 0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00,
    0x2c, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
    0x42, 0xfa, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
    0x31, 0x2e, 0x31, 0x34, 0x2e, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x2c, 0xfa, 0xff, 0xff, 0x30, 0xfa, 0xff, 0xff,
    0x34, 0xfa, 0xff, 0xff, 0x38, 0xfa, 0xff, 0xff, 0x6e, 0xfa, 0xff, 0xff,
    0x04, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x4a, 0x00, 0x00, 0x00,
    0x88, 0xff, 0xff, 0xff, 0x6f, 0x00, 0x00, 0x00, 0x86, 0xfa, 0xff, 0xff,
    0x04, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xa8, 0xa3, 0xa9, 0x55,
    0x2a, 0xfe, 0x31, 0x3c, 0xa8, 0xf4, 0x61, 0xf7, 0x52, 0xba, 0x23, 0xff,
    0xfc, 0x1b, 0x43, 0x0e, 0x81, 0xf6, 0x09, 0x38, 0x5c, 0x1b, 0xd6, 0xf7,
    0x23, 0xc7, 0x00, 0x00, 0xb2, 0xfa, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00,
    0x28, 0x00, 0x00, 0x00, 0x3a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xc8, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x82, 0x00, 0x00, 0x00,
    0x09, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0xe8, 0xff, 0xff, 0xff,
    0xd1, 0xff, 0xff, 0xff, 0xc9, 0xff, 0xff, 0xff, 0xe6, 0xfa, 0xff, 0xff,
    0x04, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xc8, 0x25, 0x01, 0x13,
    0x58, 0xd9, 0xe8, 0x0f, 0xb2, 0xc8, 0xc8, 0xcf, 0x2f, 0x5e, 0xe8, 0x01,
    0xcd, 0x0c, 0xe6, 0xfd, 0xc6, 0x0d, 0xd3, 0x22, 0x16, 0x08, 0x3e, 0xd9,
    0xe0, 0xc0, 0x0e, 0xbb, 0xeb, 0xe5, 0xbc, 0xb3, 0xdc, 0x3a, 0x2b, 0xb4,
    0xbb, 0x37, 0x0e, 0xce, 0x28, 0xf8, 0x25, 0xd9, 0x36, 0x36, 0xc6, 0xd8,
    0x1c, 0x9e, 0x1c, 0xe3, 0xdb, 0x57, 0xb1, 0xe1, 0xc5, 0xf0, 0xca, 0xb7,
    0xf3, 0x29, 0x38, 0x43, 0x4e, 0xbf, 0x44, 0x22, 0x02, 0xc7, 0xeb, 0x57,
    0x13, 0x19, 0x17, 0xc5, 0x0d, 0x01, 0x19, 0xb6, 0xa0, 0x4c, 0xe1, 0xfe,
    0xb6, 0x77, 0x32, 0xa9, 0xe1, 0x32, 0xf2, 0x11, 0xf4, 0xb1, 0x53, 0x37,
    0x36, 0x4b, 0xc7, 0x0a, 0xa7, 0xef, 0x03, 0xec, 0x55, 0xaf, 0xa9, 0x54,
    0xbc, 0x2a, 0xd3, 0x46, 0xa8, 0xf9, 0xb3, 0x44, 0x47, 0xd4, 0xd3, 0x40,
    0x44, 0xe5, 0x0b, 0x26, 0x42, 0x4a, 0xb5, 0x34, 0x81, 0x02, 0xfa, 0xf8,
    0x34, 0xed, 0x16, 0xc4, 0xd0, 0x49, 0x98, 0xa9, 0xf1, 0xf5, 0xe6, 0xba,
    0x5c, 0x33, 0x22, 0x51, 0xc8, 0xfa, 0xec, 0xde, 0xed, 0x0e, 0xd6, 0x1a,
    0xc3, 0x4f, 0x1c, 0x2f, 0x51, 0x01, 0x32, 0xae, 0xe0, 0xbf, 0x27, 0x41,
    0x50, 0x53, 0xf2, 0x2a, 0xb6, 0x50, 0x0a, 0xfc, 0xe8, 0x3c, 0x04, 0x4e,
    0xcc, 0x44, 0x3f, 0xd3, 0x45, 0xe4, 0xea, 0x1b, 0x9f, 0xe1, 0xb8, 0x13,
    0x66, 0x53, 0xf0, 0x47, 0xba, 0xfb, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00,
    0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf7, 0xff, 0xff, 0xff,
    0xf9, 0xff, 0xff, 0xff, 0xea, 0xff, 0xff, 0xff, 0x18, 0x00, 0x00, 0x00,
    0xe3, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xf5, 0xff, 0xff, 0xff,
    0x05, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00, 0xed, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x05, 0x00, 0x00, 0x00,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0xf3, 0xff, 0xff, 0xff, 0xf5, 0xff, 0xff, 0xff,
    0x16, 0xfc, 0xff, 0xff, 0x04, 0x00, 0x00, 0x00, 0x48, 0x03, 0x00, 0x00,
    0xd0, 0xd3, 0xd9, 0x08, 0xf5, 0x06, 0xc9, 0xd1, 0xcc, 0x0a, 0x3a, 0xe0,
    0xd4, 0x39, 0x1b, 0x3f, 0x3b, 0xeb, 0x2c, 0xd7, 0x38, 0xc6, 0x3c, 0xd1,
    0xdb, 0xda, 0xc5, 0x1a, 0xe7, 0xe9, 0xd7, 0xc7, 0x0c, 0x39, 0x13, 0xe5,
    0xdf, 0x2b, 0x12, 0xfa, 0x2c, 0x3f, 0xf2, 0x2f, 0x2d, 0xe2, 0x05, 0x2e,
    0x15, 0x46, 0xec, 0x21, 0x2c, 0xe1, 0xe9, 0xc6, 0xeb, 0xd2, 0xee, 0xcf,
    0x34, 0x1f, 0x07, 0xe1, 0xa2, 0x0f, 0x17, 0xd7, 0x09, 0x19, 0xef, 0xed,
    0x2f, 0x42, 0xbf, 0xee, 0xc2, 0x42, 0xbc, 0x55, 0x30, 0xde, 0xd7, 0x05,
    0xc6, 0x83, 0xa7, 0xf8, 0xda, 0xea, 0x9e, 0xe6, 0xad, 0x33, 0xf5, 0xda,
    0xb8, 0xe1, 0x41, 0x81, 0x1e, 0x06, 0x43, 0xd1, 0x52, 0x0c, 0xb7, 0x51,
    0xeb, 0x33, 0x08, 0xe8, 0x03, 0xd0, 0x0c, 0x34, 0xbf, 0x17, 0xdd, 0x23,
    0xce, 0x3a, 0xe5, 0xd5, 0x34, 0xf7, 0xad, 0x0e, 0xfa, 0x11, 0x05, 0x03,
    0xbf, 0x14, 0x2c, 0x1a, 0x04, 0xc0, 0xfe, 0x14, 0x09, 0xf3, 0xdf, 0xef,
    0x38, 0x1e, 0xbd, 0xd1, 0x0d, 0x29, 0xf2, 0x30, 0xf7, 0xdf, 0x31, 0x12,
    0x0c, 0x2d, 0xfa, 0xbe, 0xe6, 0x2d, 0xf6, 0x19, 0xcb, 0xf9, 0xf6, 0xe8,
    0xf4, 0x4e, 0xe5, 0x2f, 0x19, 0x3b, 0xbd, 0x09, 0x3b, 0xe5, 0x35, 0x0e,
    0x01, 0xfc, 0xe9, 0x2f, 0xcb, 0xfe, 0x20, 0x17, 0x1f, 0xed, 0xa2, 0x4d,
    0x0d, 0xe7, 0x37, 0x1b, 0x3e, 0x24, 0x0f, 0xf1, 0xe4, 0x22, 0xdf, 0xf3,
    0x33, 0x3e, 0xe5, 0xe6, 0x1a, 0xcb, 0xf6, 0xaa, 0xd7, 0xd2, 0xe0, 0xfe,
    0xf8, 0xf9, 0xc9, 0x04, 0xd0, 0xbf, 0x23, 0xce, 0x35, 0xcd, 0x1c, 0x15,
    0x27, 0x37, 0xf3, 0xc9, 0x0b, 0xfc, 0xc8, 0x13, 0x07, 0xe8, 0x1f, 0xf1,
    0x41, 0x2d, 0xf4, 0x32, 0xed, 0x32, 0xca, 0x24, 0x34, 0xd4, 0xfa, 0x0f,
    0xd0, 0x3a, 0xf8, 0xe9, 0xe7, 0x13, 0x13, 0xf1, 0xe2, 0x23, 0xcb, 0xcf,
    0xc0, 0xd2, 0x38, 0x0f, 0xf3, 0x1f, 0x11, 0xe9, 0x0c, 0xf8, 0x3b, 0xcf,
    0x10, 0x01, 0xe0, 0x0b, 0x10, 0xee, 0x1c, 0xed, 0xd8, 0xfb, 0xe9, 0xe5,
    0x0f, 0x2c, 0x06, 0xe7, 0x33, 0x02, 0xe6, 0x3e, 0xd9, 0xec, 0xc6, 0x37,
    0x05, 0xbf, 0x30, 0xd3, 0x23, 0xd8, 0xcd, 0xdf, 0xd7, 0xfe, 0x01, 0xc8,
    0xff, 0xe9, 0xe7, 0xc9, 0xc2, 0xf1, 0xf1, 0xc3, 0x28, 0x0f, 0x13, 0x31,
    0xcc, 0x31, 0xc2, 0xda, 0x02, 0x0f, 0xc5, 0x10, 0x17, 0x11, 0x03, 0xc1,
    0x47, 0x41, 0x3a, 0xe1, 0x46, 0x38, 0x0e, 0xf9, 0x2f, 0xb4, 0xb4, 0x4d,
    0x16, 0xf6, 0xa9, 0xfa, 0xdf, 0xe4, 0x26, 0xf7, 0xe2, 0xe3, 0x27, 0xa1,
    0xde, 0x3d, 0xca, 0x0d, 0x31, 0xdb, 0xb8, 0x15, 0xec, 0x1d, 0x2b, 0x0e,
    0x08, 0xdf, 0xc5, 0x12, 0x22, 0xcd, 0x1b, 0xff, 0x0b, 0x30, 0xdb, 0x12,
    0x17, 0xe1, 0xf9, 0x02, 0xf7, 0xf9, 0xf4, 0x0e, 0xf6, 0xce, 0x04, 0x32,
    0x38, 0xd3, 0x24, 0xe6, 0x31, 0x06, 0x33, 0xe0, 0xfd, 0xe9, 0x3c, 0x08,
    0xb8, 0x13, 0x0d, 0xe2, 0x06, 0x42, 0xeb, 0x08, 0x01, 0x3e, 0xe6, 0xf7,
    0xad, 0xa0, 0xbc, 0xbb, 0x28, 0xd5, 0x00, 0xf2, 0xfb, 0x50, 0x01, 0x28,
    0xfc, 0x28, 0xfe, 0x98, 0x15, 0x27, 0xea, 0xf7, 0xe6, 0x07, 0x21, 0xdf,
    0xdf, 0x0e, 0xcc, 0x21, 0x16, 0xd2, 0x62, 0xee, 0xbb, 0xf8, 0x36, 0x14,
    0xc2, 0x26, 0xcf, 0xf2, 0xdf, 0xfd, 0x36, 0x23, 0xfd, 0xf5, 0x07, 0x3f,
    0x10, 0x2d, 0x32, 0x3c, 0xf2, 0x35, 0x08, 0xd4, 0xd9, 0x1c, 0x09, 0x25,
    0x32, 0x2f, 0x20, 0xe7, 0xfe, 0x0f, 0x2d, 0xd3, 0xf3, 0x33, 0xd7, 0x27,
    0xc8, 0x1a, 0xd6, 0xe8, 0x2e, 0xce, 0x34, 0x1f, 0x34, 0xe5, 0xe5, 0xed,
    0x0c, 0x1a, 0x06, 0xd6, 0x2c, 0xec, 0x11, 0xde, 0xe5, 0x53, 0xfc, 0xdb,
    0xce, 0x2e, 0x3d, 0xaf, 0x3a, 0x01, 0xc3, 0xf5, 0xcb, 0xf7, 0x2a, 0x01,
    0xcd, 0xf8, 0xd1, 0x14, 0x34, 0xf5, 0x3f, 0xeb, 0xea, 0x37, 0xca, 0xe0,
    0xcc, 0xf8, 0xd2, 0x3b, 0xf2, 0x0f, 0xef, 0x18, 0xe3, 0x24, 0xff, 0xe5,
    0x0a, 0x02, 0x2e, 0x1c, 0x54, 0x36, 0x18, 0xdf, 0x1d, 0x12, 0xfc, 0xf3,
    0x1c, 0x3b, 0xd4, 0xe8, 0xe3, 0x16, 0xcc, 0x31, 0x1f, 0x08, 0x3b, 0x06,
    0xdb, 0x4d, 0x0f, 0x1a, 0x2a, 0x1f, 0x4d, 0xff, 0x2e, 0xc3, 0xe5, 0xf6,
    0xd9, 0xc0, 0x40, 0xe1, 0x1e, 0xea, 0x20, 0x27, 0x06, 0xe9, 0xc9, 0xf5,
    0xcb, 0xee, 0x06, 0xf6, 0xdd, 0xe2, 0xe4, 0xda, 0x0e, 0x1a, 0xdb, 0x17,
    0x24, 0x02, 0xd0, 0xf7, 0xe5, 0x04, 0x0a, 0xfe, 0x2e, 0x2a, 0x33, 0xc1,
    0x10, 0x06, 0xd1, 0xea, 0xff, 0xcd, 0xd8, 0x3e, 0xec, 0x16, 0xf0, 0x38,
    0xd9, 0xe0, 0xf7, 0x34, 0xda, 0x1e, 0xe6, 0xee, 0x40, 0xcf, 0xfb, 0x2c,
    0x28, 0x2c, 0x0f, 0xe4, 0x10, 0xcc, 0xe3, 0xed, 0x27, 0x17, 0x06, 0x06,
    0xc4, 0xcb, 0xc7, 0x26, 0x2a, 0xeb, 0xef, 0x28, 0xfd, 0x30, 0x36, 0xdb,
    0xf0, 0x45, 0x35, 0x1d, 0x12, 0x16, 0x15, 0xdd, 0xe1, 0x12, 0xf4, 0xd9,
    0xec, 0xd6, 0xc0, 0x43, 0xfe, 0x12, 0xc7, 0xf8, 0xd0, 0xc6, 0x13, 0x04,
    0x25, 0x23, 0xfe, 0x10, 0x40, 0x16, 0xe3, 0xf1, 0x05, 0x34, 0x17, 0x16,
    0xf4, 0xc3, 0xcd, 0x23, 0x1e, 0x4b, 0xe3, 0x04, 0xc3, 0xc9, 0x39, 0xe5,
    0x0b, 0x2e, 0xdb, 0x05, 0x06, 0x1c, 0x0e, 0xec, 0xf6, 0xed, 0x03, 0xf6,
    0xde, 0x33, 0xc5, 0xee, 0x24, 0xcc, 0x39, 0xf8, 0xd3, 0x3f, 0x26, 0x22,
    0x26, 0x0b, 0x0c, 0x0f, 0xc6, 0xc4, 0x3a, 0x0b, 0xcb, 0x0c, 0xce, 0x29,
    0xd1, 0xf4, 0x32, 0x32, 0x22, 0x15, 0xfa, 0xcd, 0xe4, 0xf1, 0xc2, 0xcf,
    0x2a, 0xf3, 0xdd, 0x3b, 0xe2, 0xee, 0xc4, 0xef, 0x21, 0xc3, 0xef, 0x32,
    0xd1, 0xc5, 0x2b, 0xe9, 0x06, 0x0a, 0x23, 0x12, 0x2f, 0x01, 0x15, 0x21,
    0x28, 0xf0, 0x07, 0x21, 0xe4, 0xba, 0xe3, 0xfd, 0x0f, 0x16, 0x05, 0xf6,
    0xfb, 0xc0, 0x39, 0x4a, 0xea, 0x2a, 0xf6, 0xe8, 0x32, 0x0b, 0x18, 0xc0,
    0x0e, 0xf3, 0xeb, 0x21, 0x4e, 0xcb, 0x27, 0x37, 0xfb, 0x39, 0xc5, 0xba,
    0xcb, 0xcd, 0x3c, 0x1e, 0x12, 0x06, 0x25, 0xeb, 0x1d, 0xed, 0xca, 0xe2,
    0x38, 0xff, 0xff, 0xff, 0x3c, 0xff, 0xff, 0xff, 0x0f, 0x00, 0x00, 0x00,
    0x4d, 0x4c, 0x49, 0x52, 0x20, 0x43, 0x6f, 0x6e, 0x76, 0x65, 0x72, 0x74,
    0x65, 0x64, 0x2e, 0x00, 0x01, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0e, 0x00, 0x18, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00,
    0x08, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x1c, 0x00, 0x00, 0x00, 0x28, 0x01, 0x00, 0x00, 0x2c, 0x01, 0x00, 0x00,
    0x30, 0x01, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x6d, 0x61, 0x69, 0x6e,
    0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0xd0, 0x00, 0x00, 0x00,
    0x88, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x0e, 0x00, 0x1a, 0x00, 0x14, 0x00, 0x10, 0x00, 0x0c, 0x00,
    0x0b, 0x00, 0x04, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x09, 0x1c, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x08, 0x00, 0x04, 0x00,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3f, 0x01, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
    0x96, 0xff, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
    0x10, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x04, 0x00, 0x04, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    0x06, 0x00, 0x00, 0x00, 0xca, 0xff, 0xff, 0xff, 0x10, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x08, 0x10, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0xba, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x08, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00,
    0x16, 0x00, 0x00, 0x00, 0x10, 0x00, 0x0c, 0x00, 0x0b, 0x00, 0x04, 0x00,
    0x0e, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08,
    0x18, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00,
    0x08, 0x00, 0x07, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x78, 0x04, 0x00, 0x00,
    0x00, 0x04, 0x00, 0x00, 0xac, 0x03, 0x00, 0x00, 0x40, 0x03, 0x00, 0x00,
    0xe4, 0x02, 0x00, 0x00, 0x78, 0x02, 0x00, 0x00, 0x24, 0x02, 0x00, 0x00,
    0x90, 0x01, 0x00, 0x00, 0xf4, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0xc2, 0xfb, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00,
    0x34, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
    0x48, 0x00, 0x00, 0x00, 0xb4, 0xfb, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x80, 0x3b, 0x19, 0x00, 0x00, 0x00, 0x53, 0x74, 0x61, 0x74,
    0x65, 0x66, 0x75, 0x6c, 0x50, 0x61, 0x72, 0x74, 0x69, 0x74, 0x69, 0x6f,
    0x6e, 0x65, 0x64, 0x43, 0x61, 0x6c, 0x6c, 0x3a, 0x30, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x2a, 0xfc, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x60, 0x00, 0x00, 0x00,
    0x1c, 0xfc, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0xb1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xb9, 0x93, 0x97, 0x3d,
    0x32, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69,
    0x61, 0x6c, 0x2f, 0x79, 0x5f, 0x70, 0x72, 0x65, 0x64, 0x2f, 0x4d, 0x61,
    0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74,
    0x69, 0x61, 0x6c, 0x2f, 0x79, 0x5f, 0x70, 0x72, 0x65, 0x64, 0x2f, 0x42,
    0x69, 0x61, 0x73, 0x41, 0x64, 0x64, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0xaa, 0xfc, 0xff, 0xff,
    0x14, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x09, 0x78, 0x00, 0x00, 0x00, 0x9c, 0xfc, 0xff, 0xff,
    0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00,
    0x7d, 0x67, 0x08, 0x3e, 0x4c, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75,
    0x65, 0x6e, 0x74, 0x69, 0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65,
    0x5f, 0x31, 0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x3b, 0x73, 0x65,
    0x71, 0x75, 0x65, 0x6e, 0x74, 0x69, 0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e,
    0x73, 0x65, 0x5f, 0x31, 0x2f, 0x52, 0x65, 0x6c, 0x75, 0x3b, 0x73, 0x65,
    0x71, 0x75, 0x65, 0x6e, 0x74, 0x69, 0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e,
    0x73, 0x65, 0x5f, 0x31, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41, 0x64, 0x64,
    0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x42, 0xfd, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00,
    0x30, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
    0x70, 0x00, 0x00, 0x00, 0x34, 0xfd, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0xe6, 0xc2, 0x62, 0x3e,
    0x46, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69,
    0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x4d, 0x61, 0x74,
    0x4d, 0x75, 0x6c, 0x3b, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69,
    0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x52, 0x65, 0x6c,
    0x75, 0x3b, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69, 0x61, 0x6c,
    0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x2f, 0x42, 0x69, 0x61, 0x73, 0x41,
    0x64, 0x64, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0xd2, 0xfd, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00,
    0x30, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
    0x34, 0x00, 0x00, 0x00, 0xc4, 0xfd, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x72, 0x5b, 0x61, 0x3a,
    0x0b, 0x00, 0x00, 0x00, 0x79, 0x5f, 0x70, 0x72, 0x65, 0x64, 0x2f, 0x62,
    0x69, 0x61, 0x73, 0x00, 0x01, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x22, 0xfe, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x48, 0x00, 0x00, 0x00,
    0x14, 0xfe, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xea, 0x78, 0xd3, 0x3b,
    0x18, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69,
    0x61, 0x6c, 0x2f, 0x79, 0x5f, 0x70, 0x72, 0x65, 0x64, 0x2f, 0x4d, 0x61,
    0x74, 0x4d, 0x75, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00, 0x8a, 0xfe, 0xff, 0xff,
    0x14, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x3c, 0x00, 0x00, 0x00, 0x7c, 0xfe, 0xff, 0xff,
    0x08, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x68, 0x89, 0x8c, 0x3a, 0x0c, 0x00, 0x00, 0x00,
    0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x62, 0x69, 0x61, 0x73,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0a, 0x00, 0x00, 0x00,
    0xe2, 0xfe, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00, 0x34, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x48, 0x00, 0x00, 0x00,
    0xd4, 0xfe, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x58, 0xa8, 0x9e, 0x3b,
    0x19, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75, 0x65, 0x6e, 0x74, 0x69,
    0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65, 0x5f, 0x31, 0x2f, 0x4d,
    0x61, 0x74, 0x4d, 0x75, 0x6c, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x0a, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x4a, 0xff, 0xff, 0xff,
    0x14, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x02, 0x34, 0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0xff,
    0x08, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xb6, 0x79, 0x0b, 0x3b, 0x0a, 0x00, 0x00, 0x00, 0x64, 0x65, 0x6e, 0x73,
    0x65, 0x2f, 0x62, 0x69, 0x61, 0x73, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x9a, 0xff, 0xff, 0xff, 0x14, 0x00, 0x00, 0x00,
    0x34, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
    0x44, 0x00, 0x00, 0x00, 0x8c, 0xff, 0xff, 0xff, 0x08, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x3b, 0x6d, 0x9c, 0x3b, 0x17, 0x00, 0x00, 0x00, 0x73, 0x65, 0x71, 0x75,
    0x65, 0x6e, 0x74, 0x69, 0x61, 0x6c, 0x2f, 0x64, 0x65, 0x6e, 0x73, 0x65,
    0x2f, 0x4d, 0x61, 0x74, 0x4d, 0x75, 0x6c, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x14, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00,
    0x18, 0x00, 0x14, 0x00, 0x13, 0x00, 0x0c, 0x00, 0x08, 0x00, 0x04, 0x00,
    0x0e, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x4c, 0x00, 0x00, 0x00,
    0x0c, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x04, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x87, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x2a, 0x42, 0xe4, 0x3e,
    0x13, 0x00, 0x00, 0x00, 0x73, 0x65, 0x72, 0x76, 0x69, 0x6e, 0x67, 0x5f,
    0x64, 0x65, 0x66, 0x61, 0x75, 0x6c, 0x74, 0x5f, 0x78, 0x3a, 0x30, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
    0xf0, 0xff, 0xff, 0xff, 0x19, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x19, 0x0c, 0x00, 0x10, 0x00, 0x0f, 0x00, 0x00, 0x00,
    0x08, 0x00, 0x04, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09};
unsigned int trained_tflite_len = 3296;

#endif  // _EI_CLASSIFIER_TFLITE_TRAINED_H_
