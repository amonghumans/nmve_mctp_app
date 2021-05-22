extern "C" {
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
}

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <CLI/CLI.hpp>
#include <bull-logger.hpp>

#define I2C_SLAVE_ADDR_OFFSET 0x1000

static unsigned int mycrc32_tab[] = {
        0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
        0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
        0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
        0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
        0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
        0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
        0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
        0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
        0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
        0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
        0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
        0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
        0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
        0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
        0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
        0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
        0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
        0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
        0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
        0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
        0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
        0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
        0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
        0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
        0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
        0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
        0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
        0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
        0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
        0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
        0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
        0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
        0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
        0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
        0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
        0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
        0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
        0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
        0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
        0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
        0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
        0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
        0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

unsigned int mycrc32(unsigned int crc, const uint8_t *buf, size_t size) {
        const unsigned char *p = buf;

        crc = crc ^ ~0U;
        while (size--) crc = mycrc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
        return crc ^ ~0U;
}


static int _init_i2c(int32_t i2c_bus_number, int32_t i2c_address);
static int _init_slave_i2c(int32_t i2c_bus_number, int32_t i2c_slave_address);
static int _i2c_device_fd = -1;
static int _source_slave_address = 0x08;

int main(int argc, char *argv[])
{
    int32_t i2c_bus_number = -1;
    int32_t i2c_address = -1;
    int32_t cmd_type = -1;
    uint8_t buf[64];
    __s32 r = -1;
    struct pollfd pfd;
    struct timespec ts;
    unsigned char data[256];
    int i=0;
    int32_t packet_size = 0;
    
    CLI::App app{"Send an I2C MCTP Set EID frame"};
    app.add_option("-b,--bus", i2c_bus_number, "i2c bus number to access the FPGA");
    app.add_option("-a,--address", i2c_address, "address of the FPGA");
    app.add_option("-c,--command", cmd_type, "MCTP Control command/NVME MI Command");
    CLI11_PARSE(app, argc, argv);
    
    /* Check parameters */

    if (i2c_bus_number == -1)
    {
        bull::Logger::info("Invalid i2c_bus_number");
        return 1;
    }

    if (i2c_address == -1)
    {
        bull::Logger::info("Invalid i2c_address");
        return 1;
    }

    if (cmd_type == -1)
    {
        bull::Logger::info("Invalid cmd_type");
        return 1;
    }

    bull::Logger::info("Init I2C");

    _init_i2c(i2c_bus_number, i2c_address);

    // Init I2C slave stuff
    pfd.fd =_init_slave_i2c(i2c_bus_number, 0x08);

    if (pfd.fd < 0)
    {
        bull::Logger::err("Cannot initialize slave client");
        return -1;
    }
    
    if(cmd_type == 0){
        // Send a Set EID control command
        buf[0] = ((_source_slave_address << 1) | 1); // source slave address
        buf[1] = 0x01; // version
        buf[2] = 0x00; // destination end point
        buf[3] = 0xAA; // Source end point
        buf[4] = 0xC9; // SOM + EOM + TO + tag
        buf[5] = 0x00; // MSG type
        buf[6] = 0x81; // Rq + ID
        buf[7] = 0x01; // Command code
        buf[8] = 0x00; // Set EID code
        buf[9] = 0xBB; //EID
        packet_size = 10;
        bull::Logger::info("Sending MCTP Control Command");
    }
    else if(cmd_type == 1){
        buf[0] = ((_source_slave_address << 1) | 1); // source slave address
        buf[1] = 0x01; // version
        buf[2] = 0x00; // destination end point
        buf[3] = 0xAA; // Source end point
        buf[4] = 0xC9; // SOM + EOM + TO + tag

        buf[5] = 0x84; // MSG type NVME
        buf[6] = 0x08; //NVME_MI_HDR_MESSAGE_TYPE_MI_COMMAND 
        buf[7] = 0x00; //Reserved
        buf[8] = 0x00; //Reserved
        buf[9] = 0x01; //NVME_MI_OPCODE_HEALTH_STATUS_POLL
        buf[10] = 0x00; //Reserved
        buf[11] = 0x00; //Reserved
        buf[12] = 0x00; //Reserved
        buf[13] = 0x00; //DWORD0
        buf[14] = 0x00; //DWORD0
        buf[15] = 0x00; //DWORD0
        buf[16] = 0x00; //DWORD0
        buf[17] = 0x00; //DWORD1
        buf[18] = 0x00; //DWORD1
        buf[19] = 0x00; //DWORD1
        buf[20] = 0x00; //DWORD1

        unsigned int integrity = 0;
        integrity = mycrc32(0,buf,20);
        /*Integrity Check*/
        buf[21] = integrity & 0xff;
        buf[22] = (integrity >> 8) & 0xff;
        buf[23] = (integrity >> 16) & 0xff;
        buf[24] = (integrity >> 24) & 0xff;
        packet_size = 25;
        bull::Logger::info("Sending NVME MI Command");
    }


    r = i2c_smbus_write_block_data(_i2c_device_fd, 0x0F, packet_size, buf);

    if ( r < 0)
    {
        std::string msg = "Cannot send block data, errno: " + std::to_string(r);
        bull::Logger::err(msg);
    }

    pfd.events = POLLPRI;
    
    //wait for answer
    while(1)
    {
        r = poll(&pfd, 1, 5000);

        if (r < 0)
            break;

        if (r == 0 || !(pfd.revents & POLLPRI))
        {
            bull::Logger::info("Timeout waiting for MCTP answer");
            break;
        }

        lseek(pfd.fd, 0, SEEK_SET);
        r = read(pfd.fd, data, sizeof(data));
        
        if (r <= 0)
            continue;

        clock_gettime(CLOCK_MONOTONIC, &ts);
        printf("[%ld.%.9ld] :", ts.tv_sec, ts.tv_nsec);
        for (i = 0; i < r; i++)
            printf(" %02x", data[i]);
        printf("\n");
        
        bull::Logger::info("Answer received, end the waiting loop"); 
        break;
    }
    close(_i2c_device_fd);
#if 0    
    if (_i2c_device_fd != -1)
    {
        close(_i2c_device_fd);
    }
#endif
    return 0;
}

static int _init_i2c(int32_t i2c_bus_number, int32_t i2c_address)
{
    int ret = 0;
    std::string msg;
    std::string i2c_device_path = "/dev/i2c-" + std::to_string(i2c_bus_number);
    msg = "Opening device: " + i2c_device_path;

    bull::Logger::info(msg);

    _i2c_device_fd = open(i2c_device_path.c_str(), O_RDWR);
    if (_i2c_device_fd == -1) {
        msg = "Cannot open device: " + i2c_device_path;
        bull::Logger::info(msg);
        return 1;
    }

    ret = ioctl(_i2c_device_fd, I2C_SLAVE, i2c_address);
    if (ret < 0) {
        msg = "Cannot communicate with I2C slave: " + std::to_string(i2c_address);
        bull::Logger::info(msg);
        close(_i2c_device_fd);
        _i2c_device_fd = -1;
        return 1;
    }
    
    ret = ioctl(_i2c_device_fd, I2C_PEC, 1);

    if (ret < 0) {
        msg = "Cannot enable PEC: " + std::to_string(i2c_address);
        bull::Logger::info(msg);
        close(_i2c_device_fd);
        _i2c_device_fd = -1;
        return 1;
    }

    bull::Logger::info("I2C initialisation success");

    return 0;
}

static int _init_slave_i2c(int32_t i2c_bus_number, int32_t i2c_slave_address)
{
    int ret = 0;
    int fd, slave_fd;
    int32_t translated_slave_address = i2c_slave_address + I2C_SLAVE_ADDR_OFFSET;
    char slave_name[64];
    char slave_file[64];
    std::string add_slave_path = "/sys/bus/i2c/devices/i2c-" + std::to_string(i2c_bus_number) + "/new_device";
    sprintf(slave_name, "slave-mqueue 0x%x", translated_slave_address);
    sprintf(slave_file, "/sys/bus/i2c/devices/%d-%x/slave-mqueue", i2c_bus_number, translated_slave_address);
    
    fd = open(add_slave_path.c_str(), O_WRONLY);

    if (fd < 0)
    {
        std::string err_msg = "Cannot open " + add_slave_path;
        bull::Logger::err(err_msg);
    }
      
    write(fd, slave_name, strlen(slave_name) + 1);
    
    slave_fd = open(slave_file, O_RDONLY);
    
    if (slave_fd < 0)
    {
        std::string err_msg = "Cannot open " + std::string(slave_file);
        bull::Logger::err(err_msg);
    }    

    return slave_fd;
}

// vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

