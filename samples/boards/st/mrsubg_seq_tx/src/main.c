#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   5000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define DATABUFFERS_SIZE 8

SMRSubGConfig_t MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields_t MRSUBG_PacketSettingsStruct;

/* USER CODE BEGIN PV */
__attribute__((aligned(4))) volatile uint8_t databuf0[DATABUFFERS_SIZE];
__attribute__((aligned(4))) volatile uint8_t databuf1[DATABUFFERS_SIZE];

static volatile uint8_t ack_failed = 0, ack_received = 0, sequencer_timeout = 0;
static uint32_t msg_count = 0;

static MRSubG_Sequencer_GlobalConfiguration_t globalcfg;
static MRSubG_Sequencer_ActionConfiguration_t actioncfgSendMsg;
static MRSubG_Sequencer_ActionConfiguration_t actioncfgWaitACK;

static void radio_driver_init(void);
static void radio_sequencer_init(void);

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	radio_driver_init();

	radio_sequencer_init();

	printf("Triggered sequencer, transmitting message and auto-waiting for ACK.\r\n");

	uint8_t restart_sequence = 0;

	while (1) {

		ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			return 0;
		}

		ret = gpio_pin_set_dt(&led, led_state);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;

		if (ack_received) {
			printf("ACK OK, scheduling next message!\r\n");
			ack_received = 0;
			restart_sequence = 1;
		}
		if (ack_failed) {
			printf("ACK missed, scheduling next message!\r\n");
			ack_failed = 0;
			restart_sequence = 1;
		}
		if (sequencer_timeout) {
			sequencer_timeout = 0;
			printf("Unexpected sequencer timeout!\r\n");
			restart_sequence = 1;
		}

		if (restart_sequence) {
			msg_count++;
			memcpy((uint8_t*)databuf0, &msg_count, sizeof(msg_count));
			globalcfg.ActionConfigPtr = &actioncfgSendMsg;
			__HAL_MRSUBG_SEQ_TRIGGER();
		}

		k_msleep(SLEEP_TIME_MS);

	}
	return 0;
}

/**
  * @brief MRSUBG Initialization Function
  * @param None
  * @retval None
  */
static void radio_driver_init(void)
{
	/** Configures the radio parameters
	 */
	MRSUBG_RadioInitStruct.lFrequencyBase = 868000000;
	MRSUBG_RadioInitStruct.xModulationSelect = MOD_2FSK;
	MRSUBG_RadioInitStruct.lDatarate = 38400;
	MRSUBG_RadioInitStruct.lFreqDev = 20000;
	MRSUBG_RadioInitStruct.lBandwidth = 100000;
	MRSUBG_RadioInitStruct.dsssExp = 0;
	MRSUBG_RadioInitStruct.outputPower = 14;
	MRSUBG_RadioInitStruct.PADrvMode = PA_DRV_TX_HP;
	HAL_MRSubG_Init(&MRSUBG_RadioInitStruct);

	/** Configures the packet parameters
	 */
	MRSUBG_PacketSettingsStruct.PreambleLength = 16;
	MRSUBG_PacketSettingsStruct.PostambleLength = 0;
	MRSUBG_PacketSettingsStruct.SyncLength = 31;
	MRSUBG_PacketSettingsStruct.SyncWord = 0x88888888;
	MRSUBG_PacketSettingsStruct.FixVarLength = VARIABLE;
	MRSUBG_PacketSettingsStruct.PreambleSequence = PRE_SEQ_0101;
	MRSUBG_PacketSettingsStruct.PostambleSequence = POST_SEQ_0101;
	MRSUBG_PacketSettingsStruct.CrcMode = PKT_CRC_MODE_8BITS;
	MRSUBG_PacketSettingsStruct.Coding = CODING_NONE;
	MRSUBG_PacketSettingsStruct.DataWhitening = ENABLE;
	MRSUBG_PacketSettingsStruct.LengthWidth = BYTE_LEN_1;
	MRSUBG_PacketSettingsStruct.SyncPresent = ENABLE;
	HAL_MRSubG_PacketBasicInit(&MRSUBG_PacketSettingsStruct);

}

/**
  * @brief MRSUBG Sequencer Initialization Function
  * @param None
  * @retval None
  */
static void radio_sequencer_init(void)
{
	/* Assign allocated buffers to databuffer pointer registers */
	__HAL_MRSUBG_SET_DATABUFFER0_POINTER((uint32_t)&databuf0[0]);
	__HAL_MRSUBG_SET_DATABUFFER1_POINTER((uint32_t)&databuf1[0]);
	__HAL_MRSUBG_SET_DATABUFFER_SIZE(DATABUFFERS_SIZE);

	HAL_MRSubG_SetPALeveldBm(7, 14, PA_DRV_TX_HP);

	/* Set a 500ms RX Timeout */
	__HAL_MRSUBG_SET_RX_TIMEOUT(HAL_MRSubG_Sequencer_Milliseconds(500));
	HAL_MRSubG_SetRSSIThreshold(-139);

	memcpy((uint8_t*)databuf0, &msg_count, sizeof(msg_count));
	__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);
	__HAL_MRSUBG_SET_PKT_LEN(DATABUFFERS_SIZE);

	actioncfgSendMsg.NextAction1Ptr = &actioncfgWaitACK;
	actioncfgSendMsg.NextAction1Ctrl = 0;
	actioncfgSendMsg.NextAction1Mask = MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F;
	actioncfgSendMsg.NextAction1Interval = 0;

	actioncfgSendMsg.NextAction2Ptr = NULL;
	actioncfgSendMsg.NextAction2Ctrl = 0;
	actioncfgSendMsg.NextAction2Mask = SEQ_MASK_NEVER_MATCH;
	actioncfgSendMsg.NextAction2Interval = 0;

	actioncfgSendMsg.ActionTimeout = 0;
	HAL_MRSubG_Sequencer_ApplyDynamicConfig(&actioncfgSendMsg, CMD_TX);

	/*******************************/

	__HAL_MRSUBG_SET_RX_MODE(RX_NORMAL);
	__HAL_MRSUBG_SET_PKT_LEN(1);
	__HAL_MRSUBG_SET_RFSEQ_IRQ_ENABLE(MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_RX_OK_E | MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_RX_CRC_ERROR_E | MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_RX_TIMEOUT_E);

	actioncfgWaitACK.NextAction1Ptr = NULL;
	actioncfgWaitACK.NextAction1Ctrl = 0;
	actioncfgWaitACK.NextAction1Mask = MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F;
	actioncfgWaitACK.NextAction1Interval = 0;

	actioncfgWaitACK.NextAction2Ptr = NULL;
	actioncfgWaitACK.NextAction2Ctrl = 0;
	actioncfgWaitACK.NextAction2Mask = (MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F | MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F);
	actioncfgWaitACK.NextAction2Interval = 0;

	actioncfgWaitACK.ActionTimeout = HAL_MRSubG_Sequencer_Milliseconds(1000);
	HAL_MRSubG_Sequencer_ApplyDynamicConfig(&actioncfgWaitACK, CMD_RX);

	/*******************************/

	globalcfg.Flags = SEQ_FLAG_FORCERELOAD | SEQ_FLAG_CLEAREVENTS;
	globalcfg.ActionConfigPtr = &actioncfgSendMsg;
	HAL_MRSubG_Sequencer_ApplyStaticConfig(&globalcfg);

	__HAL_MRSUBG_SEQ_SET_GLOBAL_CONFIG(&globalcfg);
	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(0xffffffff);
	__HAL_MRSUBG_SEQ_TRIGGER();

}

static void radio_wkup_isr(void *args)
{

	uint32_t status = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

	if (status & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F) {
		ack_received = 1;
	}
	if (status & (MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_TIMEOUT_F |
		MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F)) {
		ack_failed = 1;
	}
	if (status & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_SEQ_F) {
		uint32_t status_details = __HAL_MRSUBG_GET_RFSEQ_STATUS_DETAIL();
		if (status_details & MR_SUBG_GLOB_STATUS_RFSEQ_STATUS_DETAIL_SEQ_ACTIONTIMEOUT_F) {
			sequencer_timeout = 1;
		}
		__HAL_MRSUBG_SET_RFSEQ_STATUS_DETAIL(status_details);
	}

	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(status);

}

static int radio_irq_init(void)
{
	irq_enable(MRSUBG_IRQn);

	IRQ_CONNECT(MRSUBG_IRQn,
				1,
				radio_wkup_isr,
				NULL,
				0);
	return 0;
}

SYS_INIT(radio_irq_init, PRE_KERNEL_2, CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);