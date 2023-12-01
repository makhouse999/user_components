HOW TO USE:

SLAVE:
create a c file to descript the device feature.
example:

'''
/* a reg for storage */
/* this reg should be 16bytes align, using #pragma pack(push, 1)  #pragma pack(pop) to ensure*/
static struct holding_reg hd_reg;

/* create a funtion to handle the specific reg callback such as HOLDING, INPUT, COIL and so on  */
static int holding_reg_cb( mb_event_group_t event, mb_param_info_t * reg_info )
{
	if(event & MB_EVENT_HOLDING_REG_WR) {
		'''
	} else if (event & MB_EVENT_HOLDING_REG_RD){
		'''
	}
}

/* fill out the register information */
static struct mb_slave_dev_desc hd_reg_desc = {
	.reg_area = {
		.address = &hd_reg,
		.size = sizeof(hd_reg),
		.start_offset = 0,
		.type = MB_PARAM_HOLDING,
	},
	.cb = holding_reg_cb,
};

/* registe the reg to the user_components layer */
mb_slave_dev_register(&hd_reg_desc);



MASTER
create a c file to descript the device feature.
example:

/* create a device description table */
static struct mb_dev_desc th_dev_desc[] = {
	{
	    { 0, STR("th_t"), STR("C"), TH_ADDR, MB_PARAM_HOLDING, TH_REG_T, 1,
                    0, PARAM_TYPE_U16, PARAM_SIZE_U16, OPTS( 0, 0, 0 ), PAR_PERMS_READ },
		th_dev_cb, {NULL},
	},
	{
	    { 0, STR("th_h"), STR("H"), TH_ADDR, MB_PARAM_INPUT, TH_REG_H, 1,
                    0, PARAM_TYPE_U16, PARAM_SIZE_U16, OPTS( 0, 0, 0 ), PAR_PERMS_READ },
		th_dev_cb, {NULL},
	},	
};

/* create a funtion to handle the specific reg callback such if necessary 
	this cb function could be use for processing data or construct a reply
*/
int th_dev_cb (mb_parameter_descriptor_t * params, uint8_t * val, char * ack_params_key, uint8_t * ack_val)
{
	switch (params->mb_reg_start) {
		case TH_REG_T:

			break;
		case TH_REG_H:

			break;
		default:
			ESP_LOGI(TAG, "no reg was found");
			break;
	}
	if(...){
		*((uint16_t *)ack_val) = 0xff00;
		strcpy(ack_params_key, "param_key");	/* indicate which param_key is selected to reply 
													the param_key should be registe in 'th_dev_desc'
													ack_val is the response content.
												*/
	}
	
	return 0;
}

/* registe the reg to the user_components layer */
mb_master_dev_register(&th_dev_desc);