#include "convert.h"

/*
    @brief: Chuyển biến kiểu float sang dạng mảng byte
    @node: Chức năng hàm được sử dụng để Chuyển biến kiểu float sang dạng mảng byte 4 phần tử
    @param data: Là một biến kiểu float cần được chuyển sang mảng byte.
    @retval: Mảng bytes.
*/
uint8_t *Convert_From_Float_To_Bytes(float data)
{
    static data_format_float_bytes input_convert;
    input_convert.data_float = data;
    return input_convert.bytes;
}

/*
    @brief: Chuyển mảng các biến kiểu bytes sang biến kiểu float
    @node: Chức năng hàm được sử dụng để Chuyển mảng các biến kiểu bytes sang biến kiểu float
    @param data0: Biến đầu tiên của mảng .
    @param data1: Biến thứ hai của mảng .
    @param data2: Biến thứ ba của mảng .
    @param data3: Biến thứ tư của mảng .
    @retval: Biến int.
*/
float Convert_From_Bytes_To_Float(uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
    data_format_float_bytes input_convert;
    input_convert.bytes[3] = data0;
    input_convert.bytes[2] = data1;
    input_convert.bytes[1] = data2;
    input_convert.bytes[0] = data3;
    return input_convert.data_float;
}

/*************************************************************************************************/
/*
    @brief: Chuyển biến kiểu int sang dạng mảng byte
    @node: Chức năng hàm được sử dụng để Chuyển biến kiểu int sang dạng mảng byte 4 phần tử
    @param data: Là một biến kiểu int cần được chuyển sang mảng byte.
    @retval: Mảng bytes.
*/
uint8_t *Convert_From_Int_To_Bytes(int data)
{
    static data_format_int_bytes input_convert;
    input_convert.data_int = data;
    return input_convert.bytes;
}

/*
    @brief: Chuyển mảng các biến kiểu bytes sang biến kiểu int
    @node: Chức năng hàm được sử dụng để Chuyển mảng các biến kiểu bytes sang biến kiểu int
    @param data0: Biến đầu tiên của mảng .
    @param data1: Biến thứ hai của mảng .
    @param data2: Biến thứ ba của mảng .
    @param data3: Biến thứ tư của mảng .
    @retval: Biến int.
*/
uint32_t Convert_From_Bytes_To_Int(uint8_t data1, uint8_t data2, uint8_t data3, uint8_t data4)
{
    data_format_int_bytes input_convert;
    input_convert.bytes[0] = data1;
    input_convert.bytes[1] = data2;
    input_convert.bytes[2] = data3;
    input_convert.bytes[3] = data4;

    return input_convert.data_int;
}
/*************************************************************************************************/
/*
    @brief: Chuyển biến kiểu uint16_t sang dạng mảng byte
    @node: Chức năng hàm được sử dụng để Chuyển biến kiểu uint16_t sang dạng mảng byte 4 phần tử
    @param data: Là một biến kiểu uint16_t cần được chuyển sang mảng byte.
    @retval: Mảng bytes.
*/
uint8_t *Convert_From_Uint16_To_Bytes(int data)
{
    static data_format_uint8_16_t input_convert;
    input_convert.data_uint16 = data;
    return input_convert.bytes;
}

/*
    @brief: Chuyển mảng các biến kiểu bytes sang biến kiểu uint16_t
    @node: Chức năng hàm được sử dụng để Chuyển mảng các biến kiểu bytes sang biến kiểu uint16_t
    @param data0: Biến đầu tiên của mảng .
    @param data1: Biến thứ hai của mảng .
    @retval: Biến Uint16_t.
*/
uint16_t Convert_From_Bytes_To_Uint16(uint8_t data1, uint8_t data2)
{
    data_format_uint8_16_t input_convert;
    input_convert.bytes[0] = data1;
    input_convert.bytes[1] = data2;

    return input_convert.data_uint16;
}

