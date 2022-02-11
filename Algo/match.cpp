//
// File: match.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 09-Feb-2022 13:12:06
//

// Include Files
#include "match.h"
#include "rt_nonfinite.h"
#include "vector_slam_gyro_data.h"
#include <cstring>

// Function Definitions

//
// function hamming_dist = match(des1, des2)
// Arguments    : const unsigned char des1[32]
//                const unsigned char des2[32]
// Return Type  : unsigned int
//
namespace SLAMALGO
{
  unsigned int match(const unsigned char des1[32], const unsigned char des2[32])
  {
    unsigned int hamming_dist;
    unsigned char x[4];
    unsigned int a;
    unsigned int b;

    //  Match binary descriptors
    // 'match:3' dist = uint32(0);
    hamming_dist = 0U;

    // 'match:4' for i = 1:4:32
    for (int i = 0; i < 8; i++) {
      int b_i;
      unsigned int v;
      unsigned int qY;
      unsigned long u;
      b_i = i << 2;

      // 'match:5' pa = typecast([des1(i+3), des1(i+2), des1(i+1), des1(i)], 'uint32'); 
      x[0] = des1[b_i + 3];
      x[1] = des1[b_i + 2];
      x[2] = des1[b_i + 1];
      x[3] = des1[b_i];
      std::memcpy((void *)&a, (void *)&x[0], (unsigned int)((size_t)1 * sizeof
        (unsigned int)));

      // 'match:6' pb = typecast([des2(i+3), des2(i+2), des2(i+1), des2(i)], 'uint32'); 
      x[0] = des2[b_i + 3];
      x[1] = des2[b_i + 2];
      x[2] = des2[b_i + 1];
      x[3] = des2[b_i];
      std::memcpy((void *)&b, (void *)&x[0], (unsigned int)((size_t)1 * sizeof
        (unsigned int)));

      // 'match:7' v = bitxor(pa, pb, 'uint32');
      v = a ^ b;

      // 'match:8' vv1 = bitshift(v, -1, 'uint32');
      // 'match:9' v = v - bitand(vv1, 0x55555555u32, 'uint32');
      qY = v - (v >> 1U & 1431655765U);
      if (qY > v) {
        qY = 0U;
      }

      // 'match:10' vv2 = bitshift(v, -2, 'uint32');
      // 'match:11' v = bitand(v, 0x33333333u32, 'uint32') + bitand(vv2, 0x33333333u32, 'uint32'); 
      v = (qY & 858993459U) + (qY >> 2U & 858993459U);

      // 'match:12' vv4 = bitshift(v, -4, 'uint32');
      // 'match:13' dist = dist + uint32(bitand(bitshift(uint64(bitand(v +vv4, 0xF0F0F0Fu32, 'uint32')) * 0x1010101u64, -24, 'uint64'), 0x3Fu64)); 
      u = (((v + (v >> 4U)) & 252645135U) * 16843009UL) >> 24UL & 63UL;
      if (u > 4294967295UL) {
        u = 4294967295UL;
      }

      qY = hamming_dist + static_cast<unsigned int>(u);
      if (qY < hamming_dist) {
        qY = MAX_uint32_T;
      }

      hamming_dist = qY;
    }

    // 'match:16' hamming_dist = dist;
    // hamming_dist = 0; %was 120+
    // for k=1:size(des1,2)
    //     hamming_dist = hamming_dist + hamming8(des1(1,k),des2(1,k));
    // end
    return hamming_dist;
  }
}

//
// File trailer for match.cpp
//
// [EOF]
//
