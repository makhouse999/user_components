#ifndef __USER_HP303S_H__
#define __USER_HP303S_H__



#define HP303S_REG_WR		(0<<7)
#define HP303S_REG_RD		(1<<7)

#define HP303S_REG_ID					0x0d
#define HP303S_REG_COEF				0x10
#define HP303S_REG_RESET			0x0c
#define HP303S_REG_MEAS_CFG		0x08
#define HP303S_REG_PRS_CFG		0x06
#define HP303S_REG_TMP_CFG		0x07
#define HP303S_REG_MEAS_CFG		0x08
#define HP303S_REG_CFG				0x09

#define HP303S_REG_TMP_B2			0x03
#define HP303S_REG_PSR_B2			0x00

#define HP303S_REG_TMP_COEF_SRCE 0x28

struct hp303s_info {
	float psr;
	float temp;
	float alt;	/* unit m */
};

enum hp303s_id{
	ID_0 = 0,
	ID_1,
	ID_2,
	ID_3,
	ID_4,
	ID_5,
	ID_6,
	ID_7,
	ID_MAX,
};


struct hp303s_params {
	enum hp303s_id id;
	
	int32_t		c0, c1, c01, c11, c20, c21, c30;
	int32_t		c00, c10;
	
	float temp;
	float psr;
	
	float psr_ori;
	float psr_delta;
};


void hp303s_init(void);
void hp303s_read_id(void);
float hp303s_calc_altitude(float psr, float temp);
struct hp303s_info * hp303s_get_info(void);

#endif

