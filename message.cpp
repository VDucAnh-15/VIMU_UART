#include <message.h>
#include <convert.h>
/*mảng này dùng để kiểm tra xem sensor chọn là loại nào*/
// static list_ADC_e sensor_list_ADC_arr[SIZE_LIST_SENSOR_ADC] = {SENSOR_NTC, SENSOR_RHEOSTAT};
// static list_I2C_e sensor_list_I2C_arr[SIZE_LIST_SENSOR_I2C] = {SENSOR_LIGTH, SENSOR_DS1307};

// /*mảng này để biết data chiếm bao nhiêu byte để thực hiện cộng với length*/
// static uint8_t length_sensor_I2C_arr[SIZE_LIST_SENSOR_ADC] = {4, 4};
// static uint8_t length_sensor_ADC_arr[SIZE_LIST_SENSOR_I2C] = {4, 4};

uint8_t Message_Create_Frame_Master(frame_Master_t DataIn, uint8_t *DataOut)
{
    uint8_t *master_datain;
    frame_Master_t *master_data;

    /*dùng con trỏ frame_sensor_t trỏ đến DataIn*/
    master_data = &DataIn;

    /* ép kiểu arr_datain về kiểu uint8_t, mục đích để chuyển từ struct sang arr* */
    master_datain = (uint8_t *)master_data;

    uint16_t count_master_data = 0;

    if (DataIn.StartFrame != START_BYTE)
    {
        return 0;
    }

    switch (DataIn.TypeMessage)
    {
    case TYPE_MASTER_START:
        DataIn.LengthData = LENGTH_MASTER_START;
        break;
    case TYPE_MASTER_ASK_DATA:
        DataIn.LengthData = LENGTH_MASTER_ASK_DATA;
        break;
    }
    DataIn.CheckFrame = Check_Sum(master_datain, (LENGTH_MASTER_DEFAULT + DataIn.LengthData));

    for (uint8_t count = 0; count < (LENGTH_MASTER_DEFAULT + DataIn.LengthData); count++)
    {
        DataOut[count] = master_datain[count];
    }
    DataOut[count_master_data] = DataIn.CheckFrame & 0xff;
    DataOut[count_master_data + 1] = ((DataIn.CheckFrame >> 8) & 0xff);

    count_master_data += 2;
    return count_master_data;
}

uint8_t Message_Detech_Frame_Master(uint8_t *Master_DataIn, frame_Master_t *Master_DataOut)
{
    frame_Master_t *frame_master_data;
    frame_master_data = (frame_Master_t *)Master_DataIn;

    uint16_t master = 0;

    Master_DataOut->StartFrame = Convert_From_Bytes_To_Uint16(Master_DataIn[master], Master_DataIn[master++]);
    master += 2;

    Master_DataOut->TypeMessage = Convert_From_Bytes_To_Uint8(Master_DataIn[master]);
    master += 1;

    Master_DataOut->LengthData = Convert_From_Bytes_To_Uint16(Master_DataIn[master], Master_DataIn[master++]);
    master += 2;

    if (Master_DataOut->TypeMessage == TYPE_MESSAGE_SENSOR_SENDDATA)
    {
        for (int count_data_master = 0; count_data_master < 4; count_data_master++)
        {
            Master_DataOut->Data[count_data_master] = Master_DataIn[master++];
        }
    }

    Master_DataOut->CheckFrame = Convert_From_Bytes_To_Uint16(Master_DataIn[master], Master_DataIn[master++]);
}

uint8_t Check_Sum(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF, pos = 0, i = 0;
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
        for (i = 8; i != 0; i--)   // Loop over each bit
        {
            if ((crc & 0x0001) != 0) // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else // Else LSB is not set
            {
                crc >>= 1; // Just shift right
            }
        }
    }
    return crc;
}