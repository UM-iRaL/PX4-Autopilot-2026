#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <uORB/topics/dynamixel_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

#include <termios.h>
#include <fcntl.h>

extern "C" __EXPORT int dynamixel_driver_main(int argc, char *argv[]);


#define NUM_SERVOS 8
#define SER_PORT "/dev/ttyS1"
#define DYNAMIXEL_ENA true

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))


class Dynamixel : public ModuleBase, public ModuleParams
{
public:
	Dynamixel();

	virtual ~Dynamixel() = default;

	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	static int run_trampoline(int argc, char *argv[]);

	/** @see ModuleBase */
	static Dynamixel *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	unsigned short updateCRC(uint16_t crc_accum, uint8_t *data_blk_ptr, uint16_t data_blk_size);

	int uart_init();

	void add_stuffing(uint8_t *packet);

	void send_packet(uint8_t *txpacket);

	void reboot(uint8_t id);

	void send_command(uint8_t id, uint16_t reg, uint32_t value, uint8_t len);

	void configure_servos(void);

	void read_bytes(void);

	void process_packet(const uint8_t *pkt, uint8_t length);

	void update();

	/** @see ModuleBase::run() */
	void run() override;

	void detect_servos(void);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);


	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	// uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	struct dynamixel_controls_s dynamixel_command;

	struct termios _uart_config;
	int _serial_fd;

	uint32_t servo_mask;
	uint8_t detection_count;
	uint8_t configured_servos;
	uint8_t rebooted_servos;
	
	bool initialised;

	uint8_t pktbuf[64];
    	uint8_t pktbuf_ofs;


	uint32_t us_per_byte;
    	uint32_t us_gap;
    	uint64_t delay_time_us;

	bool _kill_switch_active{false};
	actuator_armed_s _armed_state{};

};

