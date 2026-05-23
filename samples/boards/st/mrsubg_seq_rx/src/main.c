#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#define DATABUFFERS_SIZE 8

SMRSubGConfig_t MRSUBG_RadioInitStruct;
MRSubG_PcktBasicFields_t MRSUBG_PacketSettingsStruct;

/* USER CODE BEGIN PV */
__attribute__((aligned(4))) volatile uint8_t databuf0[DATABUFFERS_SIZE];
__attribute__((aligned(4))) volatile uint8_t databuf1[DATABUFFERS_SIZE];

static MRSubG_Sequencer_GlobalConfiguration_t globalcfg;
static MRSubG_Sequencer_ActionConfiguration_t actioncfgWaitMsg;
static MRSubG_Sequencer_ActionConfiguration_t actioncfgSendACK;
static volatile uint8_t ack_sent = 0;

static void radio_driver_init(void);
static void radio_sequencer_init(void);

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

	printf("Triggered auto-ACK sequencer, waiting for message!\r\n");

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

		if (ack_sent) {
			ack_sent = 0;
			globalcfg.ActionConfigPtr = &actioncfgWaitMsg;
			__HAL_MRSUBG_SEQ_TRIGGER();

			printf("Received message: ");
			for (uint8_t i = 0; i < DATABUFFERS_SIZE; ++i)
				printf("%02x ", databuf0[i]);
			printf("\r\n");
			printf("RX-ACK procedure complete, restarting!\r\n");
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

	/* No RX timeout (timeout handled by sequencer), -139dBm sensitivity */
	__HAL_MRSUBG_SET_RX_TIMEOUT(0);
	HAL_MRSubG_SetRSSIThreshold(-139);

	__HAL_MRSUBG_SET_WAKEUP_OFFSET(6);

	/*******************************/

	__HAL_MRSUBG_SET_RX_MODE(RX_NORMAL);
	__HAL_MRSUBG_SET_PKT_LEN(DATABUFFERS_SIZE);

	actioncfgWaitMsg.NextAction1Ptr = &actioncfgSendACK;
	actioncfgWaitMsg.NextAction1Ctrl = 0;
	actioncfgWaitMsg.NextAction1Mask = MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F;
	actioncfgWaitMsg.NextAction1Interval = HAL_MRSubG_Sequencer_Milliseconds(50);

	/* If CRC error occurs, restart the action */
	actioncfgWaitMsg.NextAction2Ptr = &actioncfgWaitMsg;
	actioncfgWaitMsg.NextAction2Ctrl = 0;
	actioncfgWaitMsg.NextAction2Mask = (MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_CRC_ERROR_F);
	actioncfgWaitMsg.NextAction2Interval = 0;

	actioncfgWaitMsg.ActionTimeout = 0;
	HAL_MRSubG_Sequencer_ApplyDynamicConfig(&actioncfgWaitMsg, CMD_RX);

	/*******************************/

	__HAL_MRSUBG_SET_TX_MODE(TX_NORMAL);
	__HAL_MRSUBG_SET_PKT_LEN(1);

	__HAL_MRSUBG_SET_RFSEQ_IRQ_ENABLE(MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_TX_DONE_E);

	actioncfgSendACK.NextAction1Ptr = NULL;
	actioncfgSendACK.NextAction1Ctrl = 0;
	actioncfgSendACK.NextAction1Mask = MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F;
	actioncfgSendACK.NextAction1Interval = 0;

	actioncfgSendACK.NextAction2Ptr = NULL;
	actioncfgSendACK.NextAction2Ctrl = 0;
	actioncfgSendACK.NextAction2Mask = SEQ_MASK_NEVER_MATCH;
	actioncfgSendACK.NextAction2Interval = 0;

	actioncfgSendACK.ActionTimeout = 0;

	HAL_MRSubG_Sequencer_ApplyDynamicConfig(&actioncfgSendACK, CMD_TX);

	/*******************************/

	globalcfg.Flags = SEQ_FLAG_FORCERELOAD | SEQ_FLAG_CLEAREVENTS;
	globalcfg.ActionConfigPtr = &actioncfgWaitMsg;

	HAL_MRSubG_Sequencer_ApplyStaticConfig(&globalcfg);
	__HAL_MRSUBG_SEQ_SET_GLOBAL_CONFIG(&globalcfg);
	__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(0xffffffff);
	__HAL_MRSUBG_SEQ_TRIGGER();

}

static void radio_wkup_isr(void *args)
{

	uint32_t status = __HAL_MRSUBG_GET_RFSEQ_IRQ_STATUS();

	if (status & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_DONE_F) {
		__HAL_MRSUBG_CLEAR_RFSEQ_IRQ_FLAG(status);

		ack_sent = 1;
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