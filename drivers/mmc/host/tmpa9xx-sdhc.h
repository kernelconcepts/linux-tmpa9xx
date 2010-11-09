#ifndef __TMPA9XX_SDHC_H
#define __TMPA9XX_SDHC_H

void sdhc_functions_connect(void);

struct sdhc_platform_data {
	int vdd_32_33;
	int vdd_33_34;
	int isd0wp_wp_active;
	int has_4b_data;
};

#define SDHC_CARD_INSERT	1
#define SDHC_CARD_REMOVE	2
#define SDHC_REQ_TIMEOUT	3
#define SDHC_REQ_ERROR		4
#define SDHC_REQ_COMPLETED	5

int  sdhc_init(void);
void sdhc_deinit(void);

struct sdhc_connect_struct
{
	uint32_t fifo;
	struct sdhc_platform_data data;
};

int sdhc_connect(struct sdhc_connect_struct *s);

void sdhc_disconnect(void);

struct sdhc_setup_ios_struct
{
	int clock_mini;
	int sd_clock;
	int is_width_4;
};

void sdhc_setup_ios(struct sdhc_setup_ios_struct *s);

struct sdhc_prepare_data_transfer_struct
{
	int size;
	int seccnt;
};

void sdhc_prepare_data_transfer(struct sdhc_prepare_data_transfer_struct *s);

struct sdhc_response_get_16b_struct
{
	uint16_t b[2];
};

void sdhc_response_get_16b(struct sdhc_response_get_16b_struct *s);

struct sdhc_response_get_8b_struct
{
	uint8_t b[15];
};

void sdhc_response_get_8b(struct sdhc_response_get_8b_struct *s);

struct sdhc_start_cmd_execution_struct
{
	/* cmd */
	uint32_t opcode;
	uint32_t arg;
	uint32_t res_type;

	int is_data;
	int is_read;
	int is_multiblock;
};

void sdhc_start_cmd_execution(struct sdhc_start_cmd_execution_struct *s);

void sdhc_wait_data_ready(void);
void sdhc_mrq_stop(void);
void sdhc_reset(void);
int  sdhc_irq_handler(void);
int  sdhc_is_card_present(void);
int  sdhc_is_readonly(void);

#endif
