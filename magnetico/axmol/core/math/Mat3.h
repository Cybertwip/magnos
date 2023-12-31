#include "math/MathBase.h"

NS_AX_MATH_BEGIN

// both metal and GLSL3/ESSL3 mat3 is identical to mat3x4,
// so provide this helper struct to construct mat3 match with GPU
struct Mat3
{
#if AX_GLES_PROFILE != 200
    enum Index
    {
        _M00 = 0,
        _M01 = 1,
        _M02 = 2,
        _M10 = 4,
        _M11 = 5,
        _M12 = 6,
        _M20 = 8,
        _M21 = 9,
        _M22 = 10
    };
#else
    enum Index
    {
        _M00 = 0,
        _M01 = 1,
        _M02 = 2,
        _M10 = 3,
        _M11 = 4,
        _M12 = 5,
        _M20 = 6,
        _M21 = 7,
        _M22 = 8
    };
#endif
    Mat3() = default;
    Mat3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
    {
        this->set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }
    void set(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
    {
        m[_M00] = m00;
        m[_M01] = m01;
        m[_M02] = m02;
        // 0 3 = 0 * 4 + 3 = 3
        m[_M10] = m10;
        m[_M11] = m11;
        m[_M12] = m12;
        // 1 3  = 1 * 4 + 3 = 7
        m[_M20] = m20;
        m[_M21] = m21;
        m[_M22] = m22;
        // 2 3   = 2 * 4 + 3 = 11
    }
    float* operator[](size_t rowIndex)
    {
#if AX_GLES_PROFILE != 200
        assert(rowIndex < 3);
        return &m[rowIndex * 4];
#else
        assert(rowIndex < 2);
        return &m[rowIndex * 3];
#endif
    }
#if AX_GLES_PROFILE != 200
    float m[12] = {};
#else
    float m[9] = {};
#endif
};

NS_AX_MATH_END
