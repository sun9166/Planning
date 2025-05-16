#ifndef __SPI_TRANSMITTER_H__
#define __SPI_TRANSMITTER_H__

#include <iostream>
#include <string>
#include <string.h>
#include <memory>
#include <stdint.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/types.h>
#include "spi_dev.h"

namespace acu {
namespace common {

typedef struct SPIConfig
{
	std::string device_name;
	int spi_mode;
	int spi_transfer_speed;
	int spi_transfer_bits;
	SPIConfig() {
		device_name = "";
		spi_transfer_bits = 32;
		spi_transfer_speed = 10000000;
		spi_mode = SPI_CPHA;
	}

	SPIConfig(const std::string &_device_name) {
		device_name = _device_name;
		spi_transfer_bits = 32;
		spi_transfer_speed = 10000000;
		spi_mode = SPI_CPHA;
	}
} SPIConfig;

class SpiTransmitter
{
public:
	SpiTransmitter() {
		fd = -1;
	}
	~SpiTransmitter() {
		if (fd >= 0) {
			close(fd);
		}
	}

	int Init(const SPIConfig &_config) {
		config = _config;
		fd = open(config.device_name.c_str(), O_RDWR);
		if (fd < 0)
		{
			std::cout << "cannot open spi device " << config.device_name << std::endl;
			return -1;
		}
		int ret = ioctl(fd, SPI_IOC_WR_MODE, &config.spi_mode);
		if (ret == -1)
		{
			std::cout << "can't set wr spi mode " << config.spi_mode << std::endl;
			return -1;
		}
		ret = ioctl(fd, SPI_IOC_RD_MODE, &config.spi_mode);
		if (ret == -1)
		{
			std::cout << "can't set rd spi mode " << config.spi_mode << std::endl;
			return -1;
		}
		/*
		 * bits per word
		 */
		ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &config.spi_transfer_bits);
		if (ret == -1)
		{
			std::cout << "can't set wr bits per word " << config.spi_transfer_bits << std::endl;
			return -1;
		}


		ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &config.spi_transfer_bits);
		if (ret == -1)
		{
			std::cout << "can't set rd bits per word " << config.spi_transfer_bits << std::endl;
			return -1;
		}

		/*
		 * max speed hz
		 */
		ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &config.spi_transfer_speed);
		if (ret == -1)
		{
			std::cout << "can't set wr max speed hz " << config.spi_transfer_bits << std::endl;
			return -1;
		}


		ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &config.spi_transfer_speed);
		if (ret == -1)
		{
			std::cout << "can't set rd max speed hz " << config.spi_transfer_bits << std::endl;
			return -1;
		}

		std::cout << "spi_mode:" << config.spi_mode << std::endl;
		std::cout << "bits per word:" << config.spi_transfer_bits << std::endl;
		std::cout << "max speed(HZ):" << config.spi_transfer_speed << std::endl;

		return 0;
	}


	int Transmit(uint32_t const *tx, uint32_t const *rx, uint16_t len) {
		int ret;
		spi_ioc_transfer tr;
		memset(&tr, 0, sizeof(spi_ioc_transfer));
		tr.tx_buf = (unsigned long)tx;
		tr.rx_buf = (unsigned long)rx;
		tr.len = len;//TBD
		tr.speed_hz = config.spi_transfer_speed;
		tr.bits_per_word = config.spi_transfer_bits;

		if (config.spi_mode & SPI_TX_QUAD)
			tr.tx_nbits = 4;
		else if (config.spi_mode & SPI_TX_DUAL)
			tr.tx_nbits = 2;
		if (config.spi_mode & SPI_RX_QUAD)
			tr.rx_nbits = 4;
		else if (config.spi_mode & SPI_RX_DUAL)
			tr.rx_nbits = 2;
		if (!(config.spi_mode & SPI_LOOP)) {
			if (config.spi_mode & (SPI_TX_QUAD | SPI_TX_DUAL))
				tr.rx_buf = 0;
			else if (config.spi_mode & (SPI_RX_QUAD | SPI_RX_DUAL))
				tr.tx_buf = 0;
		}

		if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1)
		{
			std::cout << "cannot send spi message" << std::endl;
			return -1;
		}
		return 0;
	}

private:
	SPIConfig config;
	int fd;

};



}
}


#endif

