#include <QCoreApplication>
#include <QDebug>
#include <QElapsedTimer>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

int usb_out_descr;
uint8_t devs[2] = {0x12, 0x14};
float t, d;

/// Delay in nanoseconds
void nanodelay(uint32_t nanosecs)
{
    QElapsedTimer myTimer;
    myTimer.start();
    while (myTimer.nsecsElapsed() < nanosecs)
	;
}

/// Init teletype
void init_tty_device()
{
    // Open device file
    usb_out_descr = open("/dev/ttyUSB0", O_RDWR | O_NONBLOCK | O_NDELAY);

    if (usb_out_descr < 0)
    {
	    qDebug() << "Error " << errno << " opening device" << ": " << strerror (errno);
    }

    // Init serial options of device file
    struct termios usb_tty;

    memset (&usb_tty, 0, sizeof(usb_tty));

    if (tcgetattr(usb_out_descr, &usb_tty) != 0)
    {
	    qDebug() << "Error " << errno << " from tcgetattr: " << strerror(errno);
    }

    cfsetospeed (&usb_tty, B19200);
    cfsetispeed (&usb_tty, B19200);

    usb_tty.c_cflag     &=  ~PARENB;	// Make 8n1
    usb_tty.c_cflag     &=  ~CSTOPB;
    usb_tty.c_cflag     &=  ~CSIZE;
    usb_tty.c_cflag     |=  CS8;
    usb_tty.c_cflag     &=  ~CRTSCTS;	// no flow control
    usb_tty.c_lflag     =   0;		// no signaling chars, no echo, no canonical processing
    usb_tty.c_oflag     =   0;		// no remapping, no delays
    usb_tty.c_cc[VMIN]      =   0;	// read doesn't block
    usb_tty.c_cc[VTIME]     =   5;	// 0.5 seconds read timeout

    usb_tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    usb_tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);// turn off s/w flow ctrl
    usb_tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    usb_tty.c_oflag     &=  ~OPOST;              // make raw

    tcflush(usb_out_descr, TCIFLUSH);

    if (tcsetattr(usb_out_descr, TCSANOW, &usb_tty) != 0)
    {
	    qDebug() << "Error " << errno << " from tcsetattr";
    }
}

void close_tty_device()
{
    // Close device file
    close(usb_out_descr);
    qDebug() << "Device closed";
}

/// Read temperature
float read_temperature(uint8_t devaddr)
{
    uint8_t crc;
    uint32_t n_written;
    uint32_t n_read;
    uint8_t buf[32];
    uint32_t tout;
    float temperature = 0;

    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = devaddr;
    buf[3] = 0x00;
    buf[4] = 0x03;
    crc = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    buf[5] = crc;

    n_written = write(usb_out_descr, buf, 6);
    if (n_written != 6)
    {
	qDebug() << "Error writing: " << strerror(errno);
    }
    tcflush(usb_out_descr, TCOFLUSH);

    memset (&buf, '\0', 32);
    tout = 0;
    do
    {
	n_read = read(usb_out_descr, &buf, 32);
	tout ++;
    } while ((n_read == 4294967295) || (n_read == 0) && (tout < 10000));
    tcflush(usb_out_descr, TCIFLUSH);

    // qDebug() << n_read;
    // qDebug() << buf[0] << buf[1] << buf[2] << buf[3] << buf[4] << buf[5] << buf[6] << buf[7];

    if ((n_read != 8) || (tout == 10000))
    {
	qDebug() << "Error reading: " << strerror(errno);
    }
    else
    {
	crc = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6];
	if (crc != buf[7])
	{
	    qDebug() << "CRC ERROR!";
	}
	else
	{
	    if (buf[5] >= 0xF0)
	    {
		    temperature = (((float)buf[5]-0xF0)*256-(float)buf[6])/10;
	    }
	    else
	    {
		    temperature= (((float)buf[5])*256-(float)buf[6])/10;
		    if (temperature < 0)
			temperature = temperature * (-1);
	    }
	}
    }
    return temperature;
}

/// Triger distance sensor
void trigger_sensor(uint8_t devaddr)
{
    uint8_t crc;
    uint32_t n_written;
    uint8_t buf[32];
    uint32_t n_read;
    uint32_t tout;

    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = devaddr;
    buf[3] = 0x00;
    buf[4] = 0x01;
    crc = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    buf[5] = crc;

    n_written = write(usb_out_descr, buf, 6);
    if (n_written != 6)
    {
	qDebug() << "Error writing: " << strerror(errno);
    }
    tcflush(usb_out_descr, TCOFLUSH);
    tcflush(usb_out_descr, TCIFLUSH);

}

/// Read distance
float read_distance(uint8_t devaddr)
{
    uint8_t crc;
    uint32_t n_written;
    uint32_t n_read;
    uint8_t buf[32];
    uint32_t tout;
    float distance = 0;

    buf[0] = 0x55;
    buf[1] = 0xAA;
    buf[2] = devaddr;
    buf[3] = 0x00;
    buf[4] = 0x02;
    crc = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    buf[5] = crc;

    n_written = write(usb_out_descr, buf, 6);
    if (n_written != 6)
    {
	qDebug() << "Error writing: " << strerror(errno);
    }
    tcflush(usb_out_descr, TCOFLUSH);

    memset (&buf, '\0', 32);
    tout = 0;
    do
    {
	n_read = read(usb_out_descr, &buf, 32);
	tout ++;
    } while ((n_read == 4294967295) || (n_read == 0) && (tout < 10000));
    tcflush(usb_out_descr, TCIFLUSH);

     //qDebug() << n_read;
     //qDebug() << buf[0] << buf[1] << buf[2] << buf[3] << buf[4] << buf[5] << buf[6] << buf[7];

    if ((n_read != 8) || (tout == 10000))
    {
	qDebug() << "Error reading: " << strerror(errno);
    }
    else
    {
	crc = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5] + buf[6];
	if (crc != buf[7])
	{
	    qDebug() << "CRC ERROR!";
	}
	else
	{
	    distance = (float)(buf[5] << 8) + (float)buf[6];
	}
    }
    return distance;
}


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    init_tty_device();

    while (1) for (uint8_t i = 0; i < 2; i ++)
    {

//	qDebug() << "Device address: " << devs[i];
//	qDebug() << "Temperature (*C): " << read_temperature(devs[i]);
//	trigger_sensor(devs[i]);
//	nanodelay(400000000);
//	qDebug() << "Distance (cm): " << read_distance(devs[i]);
	t = read_temperature(devs[i]);
	trigger_sensor(devs[i]);
	nanodelay(400000000);
	d = read_distance(devs[i]);
	qDebug() << devs[i] << t << d;


    }

    close_tty_device();

    return a.exec();
}
