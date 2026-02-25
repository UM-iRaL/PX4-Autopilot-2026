#include "dynamixel_driver.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <sys/ioctl.h>
#include <drivers/drv_hrt.h>
#include <unistd.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


#define BROADCAST_ID 0xFE
#define MAX_ID 0xFC

// DXL protocol common commands
#define INST_PING          1
#define INST_READ          2
#define INST_WRITE         3
#define INST_REG_WRITE     4
#define INST_ACTION        5
#define INST_FACTORY_RESET 6
#define INST_CLEAR        16
#define INST_SYNC_WRITE  131
#define INST_BULK_READ   146

// 2.0 protocol commands
#define INST_REBOOT       8
#define INST_STATUS      85
#define INST_SYNC_READ  130
#define INST_BULK_WRITE 147

// 2.0 protocol packet offsets
#define PKT_HEADER0     0
#define PKT_HEADER1     1
#define PKT_HEADER2     2
#define PKT_RESERVED    3
#define PKT_ID          4
#define PKT_LENGTH_L    5
#define PKT_LENGTH_H    6
#define PKT_INSTRUCTION 7
#define PKT_ERROR       8
#define PKT_PARAMETER0  8

/* Macro for Control Table Value */
#define DXL_MAKEWORD(a, b)  ((uint16_t)(((uint8_t)(((uint64_t)(a)) & 0xff)) | ((uint16_t)((uint8_t)(((uint64_t)(b)) & 0xff))) << 8))
#define DXL_MAKEDWORD(a, b) ((uint32_t)(((uint16_t)(((uint64_t)(a)) & 0xffff)) | ((uint32_t)((uint16_t)(((uint64_t)(b)) & 0xffff))) << 16))
#define DXL_LOWORD(l)       ((uint16_t)(((uint64_t)(l)) & 0xffff))
#define DXL_HIWORD(l)       ((uint16_t)((((uint64_t)(l)) >> 16) & 0xffff))
#define DXL_LOBYTE(w)       ((uint8_t)(((uint64_t)(w)) & 0xff))
#define DXL_HIBYTE(w)       ((uint8_t)((((uint64_t)(w)) >> 8) & 0xff))

// register offsets
#define REG_OPERATING_MODE 11
#define   OPMODE_CURR_CONTROL    0
#define   OPMODE_VEL_CONTROL     1
#define   OPMODE_POS_CONTROL     3
#define   OPMODE_EXT_POS_CONTROL 4

#define REG_TORQUE_ENABLE  64

#define REG_STATUS_RETURN  68
#define   STATUS_RETURN_NONE 0
#define   STATUS_RETURN_READ 1
#define   STATUS_RETURN_ALL  2

#define REG_RETURN_DELAY_TIME 9
#define REG_BAUD_RATE 8

#define REG_GOAL_POSITION 116

// Baud rate values for Dynamixel
#define BAUD_9600   0
#define BAUD_57600  1
#define BAUD_115200 2
#define BAUD_1MBPS  3

//Position Control PID Values
#define REG_POSITION_P_GAIN 84
#define REG_POSITION_I_GAIN 82
#define REG_POSITION_D_GAIN 80

//Initial Configuration 
#define REG_STARTUP_CONFIG 60
#define REG_DRIVE_MODE 10

// how many times to send servo configure msgs
#define CONFIGURE_SERVO_COUNT 4

// Number of times to reboot servos before configuring
#define REBOOT_COUNT 10

// how many times to send servo detection
#define DETECT_SERVO_COUNT 4



unsigned short Dynamixel::updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i;
  static const uint16_t crc_table[256] = {0x0000,
  0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
  0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
  0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
  0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
  0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
  0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
  0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
  0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
  0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
  0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
  0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
  0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
  0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
  0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
  0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
  0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
  0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
  0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
  0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
  0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
  0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
  0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
  0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
  0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
  0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
  0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
  0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
  0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
  0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
  0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
  0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
  0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
  0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
  0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
  0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
  0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
  0x820D, 0x8207, 0x0202 };

  for (uint16_t j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}


int Dynamixel::uart_init(){

    _serial_fd = open(SER_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
	if(_serial_fd == -1) {
		PX4_ERR("Port open failed");
		return 1;
	}

	if(tcgetattr(_serial_fd, &_uart_config) < 0) {
		PX4_ERR("Serial port open unsuccessful");
	}
    // set half duplex uart
    // bool single_wire = true;
	// if (ioctl(_serial_fd, TIOCSSINGLEWIRE, single_wire ? (SER_SINGLEWIRE_ENABLED | SER_SINGLEWIRE_PUSHPULL |
	// 		SER_SINGLEWIRE_PULLDOWN) : 0) < 0) {
	// 	PX4_WARN("setting TIOCSSINGLEWIRE failed");
	// }
	_uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                    INLCR | PARMRK | INPCK | ISTRIP | IXON);

	_uart_config.c_oflag = 0;
	_uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	_uart_config.c_cflag &= ~(CSIZE | PARENB);
	_uart_config.c_cflag |= CS8;
	_uart_config.c_cc[VMIN]  = 1;
	_uart_config.c_cc[VTIME] = 0;

	if(cfsetispeed(&_uart_config, B115200) < 0 || cfsetospeed(&_uart_config, B115200) < 0) {
		PX4_ERR("Serial port baud rate set unsuccessful");
		return 1;
	}
    int err = tcsetattr(_serial_fd, TCSANOW, &_uart_config);
	if(err < 0) {
		PX4_ERR("Serial config write failed : %i ", err);
		return 1;
	}

	// Enable half-duplex mode for Dynamixel

    detection_count = 4;
    configured_servos = 0;
    servo_mask = 0x00FF;

    int baudrate = 115200;
    us_per_byte = 10 * 1e6 / baudrate;  // 86.8 µs per byte at 115200
    us_gap = 4 * 1e6 / baudrate;        // 34.7 µs inter-packet gap

	return 0;
}

int Dynamixel::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Dynamixel::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}

void Dynamixel::detect_servos(void)
{
    uint8_t txpacket[10] {};

    txpacket[PKT_ID] = BROADCAST_ID;
    txpacket[PKT_LENGTH_L] = 3;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_PING;

    send_packet(txpacket);

    // give plenty of time for replies from all servos
    delay_time_us = 1000 * us_per_byte;
    usleep(delay_time_us);
}

/*
  addStuffing() from Robotis SDK. This pads the packet as required by the protocol
*/
void Dynamixel::add_stuffing(uint8_t *packet)
{
    int packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H]);
    int packet_length_out = packet_length_in;

    if (packet_length_in < 8) {
        // INSTRUCTION, ADDR_L, ADDR_H, CRC16_L, CRC16_H + FF FF FD
        return;
    }

    uint8_t *packet_ptr;
    uint16_t packet_length_before_crc = packet_length_in - 2;
    for (uint16_t i = 3; i < packet_length_before_crc; i++) {
        packet_ptr = &packet[i+PKT_INSTRUCTION-2];
        if (packet_ptr[0] == 0xFF && packet_ptr[1] == 0xFF && packet_ptr[2] == 0xFD) {
            packet_length_out++;
        }
    }

    if (packet_length_in == packet_length_out) {
        // no stuffing required
        return;
    }

    uint16_t out_index  = packet_length_out + 6 - 2;  // last index before crc
    uint16_t in_index   = packet_length_in + 6 - 2;   // last index before crc

    while (out_index != in_index) {
        if (packet[in_index] == 0xFD && packet[in_index-1] == 0xFF && packet[in_index-2] == 0xFF) {
            packet[out_index--] = 0xFD; // byte stuffing
            if (out_index != in_index) {
                packet[out_index--] = packet[in_index--]; // FD
                packet[out_index--] = packet[in_index--]; // FF
                packet[out_index--] = packet[in_index--]; // FF
            }
        } else {
            packet[out_index--] = packet[in_index--];
        }
    }

    packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out);
    packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out);
}


void Dynamixel::send_packet(uint8_t *txpacket)
{
    add_stuffing(txpacket);

    // check max packet length
    uint16_t total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7;

    // make packet header
    txpacket[PKT_HEADER0]   = 0xFF;
    txpacket[PKT_HEADER1]   = 0xFF;
    txpacket[PKT_HEADER2]   = 0xFD;
    txpacket[PKT_RESERVED]  = 0x00;

    // add CRC16
    uint16_t crc = updateCRC(0, txpacket, total_packet_length - 2);    // 2: CRC16
    txpacket[total_packet_length - 2] = DXL_LOBYTE(crc);
    txpacket[total_packet_length - 1] = DXL_HIBYTE(crc);

    int write_res = write(_serial_fd, txpacket, total_packet_length);
    (void)write_res;

    // Optimized timing - no status packets, minimal delays
    delay_time_us = total_packet_length * us_per_byte +      // Command transmission time
                     us_gap;                                  // Minimal inter-packet gap
    usleep(delay_time_us);
}


ModuleBase::Descriptor Dynamixel::desc{task_spawn, custom_command, print_usage};

int Dynamixel::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Dynamixel::instantiate(ac, av);
	}, argc, argv);
}

int Dynamixel::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("Dynamixel driver",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      2048,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

Dynamixel *Dynamixel::instantiate(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			break;

		case 'f':
			break;

		case '?':
			break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	Dynamixel *instance = new Dynamixel();
	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Dynamixel::Dynamixel()
	: ModuleParams(nullptr)
{
}

void Dynamixel::run()
{
    int dynamixel_controls_sub = orb_subscribe(ORB_ID(dynamixel_controls));
    int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	px4_pollfd_struct_t fds[1];
	fds[0].fd = dynamixel_controls_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	// parameters_update(true);
    PX4_INFO("starting module");
	while (!should_exit()) {
        // PX4_INFO("iteration");
		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {

			orb_copy(ORB_ID(dynamixel_controls), dynamixel_controls_sub, &dynamixel_command);
            // PX4_INFO("servo:\t%d",
			// 		dynamixel_command.dynamixel_controls[0]);

		}

        // update kill switch state
        bool armed_updated = false;
        orb_check(armed_sub, &armed_updated);
        if (armed_updated) {
            orb_copy(ORB_ID(actuator_armed), armed_sub, &_armed_state);
        }

        update();
		parameters_update();
        // px4_usleep(250000);

	}

	// orb_unsubscribe(sensor_combined_sub);
}


/*
  broadcast configure all servos
 */
void Dynamixel::configure_servos(void)
{
    // disable torque control first
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);
    usleep(1000);

    // use position control mode (needs torque disabled)
    // send_command(BROADCAST_ID, REG_OPERATING_MODE, OPMODE_EXT_POS_CONTROL, 1);
    // usleep(1000);

    // disable RAM restore
    send_command(BROADCAST_ID, REG_STARTUP_CONFIG, 0, 1);
    usleep(1000);

    // set baud rate to 115200 for faster communication
    // for (uint8_t id = 1; id <= 8; id++) {
    //     send_command(id, REG_BAUD_RATE, BAUD_115200, 1);
    // }

    // set status return to none for maximum speed
    for (uint8_t id = 1; id <= 8; id++) {
        send_command(id, REG_STATUS_RETURN, STATUS_RETURN_READ, 1);
        usleep(1000);
    }

    // set return delay time to 0µs for faster response
    for (uint8_t id = 1; id <= 8; id++) {
        send_command(id, REG_RETURN_DELAY_TIME, 0, 1);
        usleep(1000);
    }

    // set PID values for servos 1-4
    for (uint8_t id = 1; id <= 4; id++) {
        send_command(id, REG_POSITION_P_GAIN, 2000, 2);
        usleep(1000);
        send_command(id, REG_POSITION_I_GAIN, 100, 2);
        usleep(1000);
        send_command(id, REG_POSITION_D_GAIN, 600, 2);
        usleep(1000);
    }

    // set PID values for servos 5-8
    for (uint8_t id = 5; id <= 8; id++) {
        send_command(id, REG_POSITION_P_GAIN, 1000, 2);
        usleep(1000);
        send_command(id, REG_POSITION_I_GAIN, 75, 2);
        usleep(1000);
        send_command(id, REG_POSITION_D_GAIN, 0, 2);
        usleep(1000);
    }

    // enable torque control
    send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 1, 1);
    usleep(1000);
}


void Dynamixel::read_bytes(void)
{
    uint8_t read_buf[64];
    uint32_t n = read(_serial_fd, &read_buf, sizeof(read_buf));

    if (n == 0 && pktbuf_ofs < PKT_INSTRUCTION) {
        return;
    }

    if (n > sizeof(pktbuf) - pktbuf_ofs) {
        n = sizeof(pktbuf) - pktbuf_ofs;
    }
    for (uint8_t i=0; i<n; i++) {
        pktbuf[pktbuf_ofs++] = read_buf[i];
    }

    // discard bad leading data. This should be rare
    while (pktbuf_ofs >= 4 &&
           (pktbuf[0] != 0xFF || pktbuf[1] != 0xFF || pktbuf[2] != 0xFD || pktbuf[3] != 0x00)) {
        memmove(pktbuf, &pktbuf[1], pktbuf_ofs-1);
        pktbuf_ofs--;
    }

    if (pktbuf_ofs < 10) {
        // not enough data yet
        return;
    }

    const uint16_t total_packet_length = DXL_MAKEWORD(pktbuf[PKT_LENGTH_L], pktbuf[PKT_LENGTH_H]) + PKT_INSTRUCTION;
    if (total_packet_length > sizeof(pktbuf)) {
        pktbuf_ofs = 0;
        return;
    }
    if (pktbuf_ofs < total_packet_length) {
        // more data needed
        return;
    }

    // check CRC
    const uint16_t crc = DXL_MAKEWORD(pktbuf[total_packet_length-2], pktbuf[total_packet_length-1]);
    const uint16_t calc_crc = updateCRC(0, pktbuf, total_packet_length - 2);
    if (calc_crc != crc) {
        memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
        pktbuf_ofs -= total_packet_length;
        return;
    }

    // process full packet
    process_packet(pktbuf, total_packet_length);

    memmove(pktbuf, &pktbuf[total_packet_length], pktbuf_ofs - total_packet_length);
    pktbuf_ofs -= total_packet_length;
}

/*
  process a packet from a servo
 */
void Dynamixel::process_packet(const uint8_t *pkt, uint8_t length)
{
    uint8_t id = pkt[PKT_ID];
    if (id > 16 || id < 1) {
        // discard packets from servos beyond max or min. Note that we
        // don't allow servo 0, to make mapping to SERVOn_* parameters
        // easier
        return;
    }
    uint32_t id_mask = (1U<<(id-1));
    if (!(id_mask & servo_mask)) {
        // mark the servo as present
        servo_mask |= id_mask;
        PX4_INFO("Robotis: new servo %u\n", id);
    }
}



void Dynamixel::update()
{
    if (!initialised) {
        initialised = true;
        uart_init();
        return;
    }

    if (detection_count < DETECT_SERVO_COUNT) {
        detection_count++;
        detect_servos();
        return;
    }

    if (servo_mask == 0) {
        return;
    }

    if (rebooted_servos < REBOOT_COUNT) {
        rebooted_servos++;
        reboot(BROADCAST_ID);
        return;
    }

    if (configured_servos < CONFIGURE_SERVO_COUNT) {
        configured_servos++;
        configure_servos();
        return;
    }

    // kill switch handling
    const bool kill_active = _armed_state.kill || _armed_state.lockdown;

    if (!_kill_switch_active && kill_active) {
        // Kill switch just engaged — disable torque immediately
        send_command(BROADCAST_ID, REG_TORQUE_ENABLE, 0, 1);
        _kill_switch_active = true;
        PX4_WARN("Kill switch engaged — servo torque disabled");
        return;
    }

    if (_kill_switch_active && !kill_active) {
        // Kill switch released — force reconfiguration
        configured_servos = 0;
        _kill_switch_active = false;
        PX4_INFO("Kill switch released — reconfiguring servos");
        return;
    }

    if (_kill_switch_active) {
        return;  // still killed, skip servo commands
    }

    // loop for all 16 channels
    for (uint8_t i=0; i<NUM_SERVOS; i++) {
        if (((1U<<i) & servo_mask) == 0) {
            continue;
        }
        // SRV_Channel *c = SRV_Channels::srv_channel(i);
        // if (c == nullptr) {
        //     continue;
        // }
        // const uint16_t pwm = c->get_output_pwm();
        // const uint16_t min = c->get_output_min();
        // const uint16_t max = c->get_output_max();
        // float v = float(pwm - min) / (max - min);
        // uint32_t value = pos_min + v * (pos_max - pos_min);
        // PX4_INFO("Sending servo %d a value of %d", i, (int)dynamixel_command.dynamixel_controls[i]);
        send_command(i+1, REG_GOAL_POSITION, dynamixel_command.dynamixel_controls[i], 4);
    }
}

void Dynamixel::reboot(uint8_t id)
{
    uint8_t txpacket[16] {};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH_L] = 0x03;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_REBOOT;

    send_packet(txpacket);
}


void Dynamixel::send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len)
{
    uint8_t txpacket[16] {};

    txpacket[PKT_ID] = id;
    txpacket[PKT_LENGTH_L] = 5 + len;
    txpacket[PKT_LENGTH_H] = 0;
    txpacket[PKT_INSTRUCTION] = INST_WRITE;
    txpacket[PKT_INSTRUCTION+1] = DXL_LOBYTE(reg);
    txpacket[PKT_INSTRUCTION+2] = DXL_HIBYTE(reg);
    memcpy(&txpacket[PKT_INSTRUCTION+3], &value, MIN(len,4));

    send_packet(txpacket);
}


void Dynamixel::parameters_update(bool force)
{
	// check for parameter updates
	// if (_parameter_update_sub.updated() || force) {
	// 	// clear update
	// 	parameter_update_s update;
	// 	_parameter_update_sub.copy(&update);

	// 	// update parameters from storage
	// 	updateParams();
	// }
}

int Dynamixel::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
        ### Description
        Section that describes the provided module functionality.

        This is a template for a module running as a task in the background with start/stop/status functionality.

        ### Implementation
        Section describing the high-level implementation of this module.

        ### Examples
        CLI usage example:
        $ module start -f -p 42

        )DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "dynamixel");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int dynamixel_driver_main(int argc, char *argv[])
{
	return ModuleBase::main(Dynamixel::desc, argc, argv);
}
