#include "hal/hal_wifi.h"
#include "hal/hal_generic.h"
#include "hal/hal_flashVars.h"
#include "hal/hal_adc.h"
#include "new_common.h"
//#include "driver/drv_ir.h"
#include "driver/drv_public.h"
#include "driver/drv_bl_shared.h"
//#include "ir/ir_local.h"
#include "driver/drv_deviceclock.h"
// Commands register, execution API and cmd tokenizer
#include "cmnds/cmd_public.h"
// overall config variables for app - like ENABLE_LITTLEFS
#include "obk_config.h"
#include "httpserver/new_http.h"
#include "httpserver/http_fns.h"
#include "new_pins.h"
#include "quicktick.h"
#include "new_cfg.h"
#include "logging/logging.h"
#include "httpserver/http_tcp_server.h"
#include "httpserver/rest_interface.h"
#include "mqtt/new_mqtt.h"
#include "hal/hal_ota.h"
#if ENABLE_LITTLEFS
#include "littlefs/our_lfs.h"
#endif
#if ENABLE_LITTLEFS && ENABLE_LOG2LFS
uint8_t g_log2lfs;
#endif
#include "driver/drv_ntp.h"
#include "driver/drv_mdns.h"
#include "driver/drv_ssdp.h"
#include "driver/drv_uart.h"
#if PLATFORM_BEKEN
#include <mcu_ps.h>
#include <fake_clock_pub.h>
#include <BkDriverWdg.h>
#include "temp_detect_pub.h"
#include "BkDriverWdg.h"
void bg_register_irda_check_func(FUNCPTR func);
extern void WFI(void);
#elif PLATFORM_BL602 && !PLATFORM_BL_NEW
#include <bl_sys.h>
#include <hosal_adc.h>
#include <bl_wdt.h>
#elif PLATFORM_W600 || PLATFORM_W800
#include "wm_watchdog.h"
#elif PLATFORM_LN882H
#include "hal/hal_wdt.h"
#include "hal/hal_gpio.h"
#elif PLATFORM_ESPIDF
#include "esp_timer.h"
#elif PLATFORM_ECR6600
#include "hal_adc.h"
#endif
int g_secondsElapsed = 0;
// open access point after this number of seconds
int g_openAP = 0;
// connect to wifi after this number of seconds
static int g_connectToWiFi = 0;
// reset after this number
static int g_reset = 0;
// is connected to WiFi?
static int g_bHasWiFiConnected = 0;
// is Open Access point or a client?
static int g_bOpenAccessPointMode = 0;
// in safe mode, user can press a button to enter the unsafe one
static int g_doUnsafeInitIn = 0;
int g_bootFailures = 0;
static int g_saveCfgAfter = 0;
int g_startPingWatchDogAfter = 60;
// many boots failed? do not run pins or anything risky
int bSafeMode = 0;
// not really <time>, but rather a loop count, but it doesn't really matter much
// start disabled.
int g_timeSinceLastPingReply = -1;
int g_prevTimeSinceLastPingReply = -1;
char g_wifi_bssid[33] = { "30:B5:C2:5D:70:72" };
uint8_t g_wifi_channel = 12;
// was it ran?
static int g_bPingWatchDogStarted = 0;
// current IP string, this is compared with IP returned from HAL
// and if it changes, the MQTT publish is done
static char g_currentIPString[32] = { 0 };
static HALWifiStatus_t g_newWiFiStatus = WIFI_UNDEFINED;
static HALWifiStatus_t g_prevWiFiStatus = WIFI_UNDEFINED;
static int g_noMQTTTime = 0;
uint8_t g_StartupDelayOver = 0;
uint32_t idleCount = 0;
int DRV_SSDP_Active = 0;
int DRV_MDNS_Active = 0;
#define LOG_FEATURE LOG_FEATURE_MAIN
void Main_ForceUnsafeInit();
#if PLATFORM_BL602 || PLATFORM_W600 || PLATFORM_W800
#define DEF_USE_WFI 1
#else
#define DEF_USE_WFI 0
#endif
#if PLATFORM_BEKEN
#define WFI_FUNC WFI
#elif PLATFORM_BL602 || PLATFORM_W600 || PLATFORM_REALTEK || PLATFORM_XRADIO || PLATFORM_W600 || PLATFORM_RDA5981 || PLATFORM_LN882H || PLATFORM_LN8825 \
	|| PLATFORM_BL_NEW || PLATFORM_GD32VW553
#define WFI_FUNC() __asm volatile("wfi")
#elif PLATFORM_W800
#define WFI_FUNC __WFI
#endif
bool g_use_wfi = DEF_USE_WFI;
// TEMPORARY
int ota_status = -1;
int total_bytes = 0;
int OTA_GetProgress()
{
	return ota_status;
}
void OTA_ResetProgress()
{
	ota_status = -1;
}
void OTA_IncrementProgress(int value)
{
	ota_status += value;
}
int OTA_GetTotalBytes()
{
	return total_bytes;
}
void OTA_SetTotalBytes(int value)
{
	total_bytes = value;
}
#if PLATFORM_XR806 || PLATFORM_XR872
size_t xPortGetFreeHeapSize()
{
	return sram_free_heap_size();
}
#elif PLATFORM_RDA5981
#include "hal/api/mbed_stats.h"
extern uint32_t mbed_heap_size;
size_t xPortGetFreeHeapSize()
{
	mbed_stats_heap_t heap_stats;
	mbed_stats_heap_get(&heap_stats);
	return mbed_heap_size - heap_stats.current_size;
}
int _kill(int pid, int sig)
{
	errno = EINVAL;
	return -1;
}
pid_t _getpid()
{
	return 1;
}
#endif
#if PLATFORM_BL602 && !PLATFORM_BL_NEW
/// Read the Internal Temperature Sensor as Float. Returns 0 if successful.
static int get_tsen_adc(
	float *temp,      //  Pointer to float to store the temperature
	uint8_t log_flag  //  0 to disable logging
) {
	*temp = hosal_adc_tsen_value_get_f(hosal_adc_device_get());
	return 0;
}
#endif
#if PLATFORM_BEKEN
#if (OBK_VARIANT == OBK_VARIANT_BATTERY)
	#if PLATFORM_BEKEN_NEW
		#define START_MS_DELAY 10;
	#else
		#define START_MS_DELAY 0;
	#endif
#else
	#define START_MS_DELAY 250;
#endif
// this function waits for the extended app functions to finish starting.
extern void extended_app_waiting_for_launch(void);
void extended_app_waiting_for_launch2()
{
#ifndef PLATFORM_BEKEN
	extended_app_waiting_for_launch();
#endif
#if PLATFORM_BK7231N || PLATFORM_BEKEN_NEW
	uint8_t startDelay = START_MS_DELAY;
	bk_printf("\r\ndelaying start\r\n");
	for(uint8_t i = 0; i < startDelay / 10; i++)
	{
		rtos_delay_milliseconds(10);
		bk_printf("#Startup delayed %dms#\r\n", i * 10);
	}
	bk_printf("\r\nstarting....\r\n");
#endif
}
#else
void extended_app_waiting_for_launch2(void) {
}
#endif
#if PLATFORM_ESPIDF || PLATFORM_REALTEK_NEW || PLATFORM_BL_NEW
int LWIP_GetMaxSockets() {
	return 0;
}
int LWIP_GetActiveSockets() {
	return 0;
}
#endif
#if PLATFORM_BL602 || PLATFORM_W800 || PLATFORM_W600 || PLATFORM_LN882H || PLATFORM_LN8825 \
	|| PLATFORM_ESPIDF || PLATFORM_TR6260 || PLATFORM_REALTEK || PLATFORM_ECR6600 \
	|| PLATFORM_XRADIO || PLATFORM_ESP8266 || PLATFORM_BL_NEW || PLATFORM_GD32VW553
OSStatus rtos_create_thread(beken_thread_t* thread,
	uint8_t priority, const char* name,
	beken_thread_function_t function,
	uint32_t stack_size, beken_thread_arg_t arg) {
	OSStatus err = kNoErr;
	err = xTaskCreate(function, name, stack_size / sizeof(StackType_t), arg, priority, thread);
	if (err == pdPASS) {
		return 0;
	}
	else if (err == errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY) {
		printf("Thread create %s - errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY\n", name);
	}
	else {
		printf("Thread create %s - err %i\n", name, err);
	}
	return 1;
}
OSStatus rtos_delete_thread(beken_thread_t* thread) {
	if(thread == NULL) vTaskDelete(NULL);
	else vTaskDelete(*thread);
	return kNoErr;
}
OSStatus rtos_suspend_thread(beken_thread_t* thread)
{
	if(thread == NULL) vTaskSuspend(NULL);
	else vTaskSuspend(*thread);
	return kNoErr;
}
#elif PLATFORM_TXW81X
OSStatus rtos_create_thread(beken_thread_t* thread,
	uint8_t priority, const char* name,
	beken_thread_function_t function,
	uint32_t stack_size, beken_thread_arg_t arg)
{
	OSStatus err = kNoErr;
	*thread = os_task_create(name, function, arg, priority, 0, NULL, stack_size);
	if(*thread != NULL)
	{
		return 0;
	}
	else
	{
		printf("Thread create %s - err %i\n", name, err);
	}
	return 1;
}
OSStatus rtos_delete_thread(beken_thread_t* thread)
{
	if(thread == NULL)
	{
		k_task_handle_t hdl = os_task_current();
		os_task_destroy(hdl);
	}
	else os_task_destroy(*thread);
	return kNoErr;
}
OSStatus rtos_suspend_thread(beken_thread_t* thread)
{
	if(thread == NULL)
	{
		k_task_handle_t hdl = os_task_current();
		os_task_suspend2(hdl);
	}
	else os_task_suspend2(*thread);
	return kNoErr;
}
#elif PLATFORM_RDA5981
#include "rt_TypeDef.h"
OSStatus rtos_create_thread(beken_thread_t thread,
	uint8_t priority, const char* name,
	beken_thread_function_t function,
	uint32_t stack_size, beken_thread_arg_t arg)
{
	osThreadDef_t def;
	osThreadId     id;
	def.pthread = (os_pthread)function;
	def.tpriority = osPriorityNormal;
	def.stacksize = stack_size;
	def.stack_pointer = malloc(stack_size);
	if(def.stack_pointer == NULL)
	{
		printf("Error allocating the stack memory");
		return 1;
	}
	thread = osThreadCreate(&def, arg);
	if(thread == NULL)
	{
		free(def.stack_pointer);
		printf("Thread create %s - err\n", name);
		return 1;
	}
	return 0;
}
OSStatus rtos_delete_thread(beken_thread_t thread)
{
	if(thread == NULL)
	{
		thread = osThreadGetId();
	}
	P_TCB tcb = rt_tid2ptcb(thread);
	uint32_t* stk = tcb->stack;
	if(stk == NULL) printf("rtos_delete_thread stk is null\n");
	free(stk);
	osThreadTerminate(thread);
	return kNoErr;
}
#endif
void MAIN_ScheduleUnsafeInit(int delSeconds) {
	g_doUnsafeInitIn = delSeconds;
}
void RESET_ScheduleModuleReset(int delSeconds) {
	g_reset = delSeconds;
}
static char scheduledDriverName[4][16];
static int scheduledDelay[4] = { -1, -1, -1, -1 };
void ScheduleDriverStart(const char* name, int delay) {
	int i;
	for (i = 0; i < 4; i++) {
		if (!strcmp(scheduledDriverName[i], name)) {
			scheduledDelay[i] = delay;
			return;
		}
	}
	for (i = 0; i < 4; i++) {
		if (scheduledDelay[i] == -1) {
			strncpy(scheduledDriverName[i], name, 16);
			scheduledDelay[i] = delay;
			return;
		}
	}
}
#if defined(PLATFORM_LN882H) || PLATFORM_LN882H
extern int g_ln882h_pendingPowerSaveCommand;
void LN882H_ApplyPowerSave(int bOn);
#endif
#if ALLOW_SSID2
#define SSID_USE_SSID1  0
#define SSID_USE_SSID2  1
static int g_SSIDactual = SSID_USE_SSID1;
static int g_SSIDSwitchAfterTry = 3;
static int g_SSIDSwitchCnt = 0;
#endif
void CheckForSSID12_Switch() {
#if ALLOW_SSID2
	if (CFG_GetWiFiSSID2()[0] == 0) return;
	if (g_SSIDSwitchCnt++ < g_SSIDSwitchAfterTry) {
		ADDLOGF_INFO("WiFi SSID: waiting for SSID switch %d/%d (using SSID%d)", g_SSIDSwitchCnt, g_SSIDSwitchAfterTry, g_SSIDactual+1);
		return;
	}
	g_SSIDSwitchCnt = 0;
	g_SSIDactual ^= 1;
	ADDLOGF_INFO("WiFi SSID: switching to SSID%i", g_SSIDactual + 1);
	if (CFG_HasFlag(OBK_FLAG_WIFI_ENHANCED_FAST_CONNECT)) HAL_DisableEnhancedFastConnect();
#endif
}
void Init_WiFiSSIDactual_FromChannelIfSet(void) {
#if ALLOW_SSID2
	g_SSIDactual = FV_GetStartupSSID_StoredValue(SSID_USE_SSID1);
#endif
}
const char* CFG_GetWiFiSSIDX() {
#if ALLOW_SSID2
	if (g_SSIDactual) {
		return CFG_GetWiFiSSID2();
	}
	else {
		return CFG_GetWiFiSSID();
	}
#else
	return CFG_GetWiFiSSID();
#endif
}
const char* CFG_GetWiFiPassX() {
#if ALLOW_SSID2
	if (g_SSIDactual) {
		return CFG_GetWiFiPass2();
	}
	else {
		return CFG_GetWiFiPass();
	}
#else
	return CFG_GetWiFiPass();
#endif
}

// ===================== UDP 继电器扩展代码 开始 =====================
#define UDP_LISTEN_PORT 8899
static uint8_t udp_task_created = 0;
static int udp_sock = -1;

void udp_relay_task(void *pvParameters)
{
	struct sockaddr_in server_addr;
	char buf[64];
	int recv_len;

	// 等待WiFi联网成功
	while (!Main_HasWiFiConnected())
	{
		rtos_delay_milliseconds(500);
	}

	udp_sock = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_sock < 0)
	{
		vTaskDelete(NULL);
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(UDP_LISTEN_PORT);
	server_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(udp_sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
	{
		close(udp_sock);
		vTaskDelete(NULL);
	}

	ADDLOGF_INFO("UDP控制服务已启动，端口：%d", UDP_LISTEN_PORT);

	while (1)
	{
		struct sockaddr_in client;
		socklen_t cli_len = sizeof(client);
		recv_len = recvfrom(udp_sock, buf, sizeof(buf)-1, 0, (struct sockaddr *)&client, &cli_len);

		if (recv_len > 0)
		{
			buf[recv_len] = '\0';
			if(strstr(buf, "ON"))
			{
				RELAY_SetState(1);
				ADDLOGF_INFO("UDP指令：继电器开启");
			}
			else if(strstr(buf, "OFF"))
			{
				RELAY_SetState(0);
				ADDLOGF_INFO("UDP指令：继电器关闭");
			}
		}
		rtos_delay_milliseconds(50);
	}
}

// WiFi连上后自动创建UDP任务
void udp_start_trigger(int wifi_code)
{
	if (wifi_code == WIFI_STA_CONNECTED && udp_task_created == 0)
	{
		udp_task_created = 1;
		rtos_create_thread(NULL, 5, "udp_relay", udp_relay_task, 2048, NULL);
	}
}
// ===================== UDP 继电器扩展代码 结束 =====================

void Main_OnWiFiStatusChange(int code)
{
	switch (code)
	{
	case WIFI_STA_CONNECTING:
		g_bHasWiFiConnected = 0;
		g_connectToWiFi = 120;
		ADDLOGF_INFO("%s - WIFI_STA_CONNECTING - %i", __func__, code);
		break;
	case WIFI_STA_DISCONNECTED:
#if PLATFORM_BEKEN
		if (g_bHasWiFiConnected != 0)
		{
			HAL_DisconnectFromWifi();
		}
#endif
		if(g_secondsElapsed < 30)
		{
			g_connectToWiFi = 5;
		}
		else
		{
			g_connectToWiFi = 15;
		}
		g_bHasWiFiConnected = 0;
		g_timeSinceLastPingReply = -1;
		ADDLOGF_INFO("%s - WIFI_STA_DISCONNECTED - %i", __func__, code);
		break;
	case WIFI_STA_AUTH_FAILED:
		if (g_secondsElapsed < 30) {
			g_connectToWiFi = 5;
		}
		else {
			g_connectToWiFi = 60;
		}
		g_bHasWiFiConnected = 0;
		ADDLOGF_INFO("%s - WIFI_STA_AUTH_FAILED - %i", __func__, code);
		break;
	case WIFI_STA_CONNECTED:
#if ALLOW_SSID2
		if (!g_bHasWiFiConnected) FV_UpdateStartupSSIDIfChanged_StoredValue(g_SSIDactual);
#endif
		g_bHasWiFiConnected = 1;
		ADDLOGF_INFO("%s - WIFI_STA_CONNECTED - %i", __func__, code);
		udp_start_trigger(code); // 联网启动UDP
#if ALLOW_SSID2
		g_SSIDSwitchCnt = 0;
#endif
		if (bSafeMode == 0) {
			HAL_GetWiFiBSSID(g_wifi_bssid);
			HAL_GetWiFiChannel(&g_wifi_channel);
			if (strlen(CFG_DeviceGroups_GetName()) > 0) {
				ScheduleDriverStart("DGR", 5);
			}
			if (DRV_SSDP_Active) {
				ScheduleDriverStart("SSDP", 5);
			}
			if (DRV_MDNS_Active) {
				ScheduleDriverStart("MDNS", 5);
			}
		}
#if defined(PLATFORM_LN882H) || PLATFORM_LN882H
		if (g_ln882h_pendingPowerSaveCommand != -1) {
			ADDLOG_INFO(LOG_FEATURE_CMD, "CMD_PowerSave: applying delayed setting. PowerSave will set to %i", g_ln882h_pendingPowerSaveCommand);
			LN882H_ApplyPowerSave(g_ln882h_pendingPowerSaveCommand);
			g_ln882h_pendingPowerSaveCommand = -1;
		}
#endif
		break;
	case WIFI_AP_CONNECTED:
		g_bHasWiFiConnected = 1;
		ADDLOGF_INFO("%s - WIFI_AP_CONNECTED - %i", __func__, code);
		break;
	case WIFI_AP_FAILED:
		g_bHasWiFiConnected = 0;
		ADDLOGF_INFO("%s - WIFI_AP_FAILED - %i", __func__, code);
		break;
	default:
		break;
	}
	g_newWiFiStatus = code;
}
void CFG_Save_SetupTimer()
{
	g_saveCfgAfter = 3;
}
void Main_OnPingCheckerReply(int ms)
{
	g_timeSinceLastPingReply = 0;
}
int g_doHomeAssistantDiscoveryIn = 0;
int g_bBootMarkedOK = 0;
int g_rebootReason = 0;
static int bMQTTconnected = 0;
int Main_HasMQTTConnected()
{
	return bMQTTconnected;
}
int Main_HasWiFiConnected()
{
	return g_bHasWiFiConnected;
}
#ifdef OBK_MCU_SLEEP_METRICS_ENABLE
extern OBK_MCU_SLEEP_METRICS OBK_Mcu_metrics;
void Main_LogPowerSave() {
	if (!OBK_Mcu_metrics.nexttask) {
		OBK_Mcu_metrics.nexttask = "unkn";
	}
	if (!OBK_Mcu_metrics.task) {
		OBK_Mcu_metrics.task = "unkn2";
	}
	ADDLOGF_DEBUG("PS: %ums/%ums longests:%ums/%ums req:%ums/%ums %s %s",
		BK_TICKS_TO_MS(OBK_Mcu_metrics.slept_ticks),
		BK_TICKS_TO_MS(OBK_Mcu_metrics.sleep_requested_ticks),
		BK_TICKS_TO_MS(OBK_Mcu_metrics.longest_sleep_1s),
		BK_TICKS_TO_MS(OBK_Mcu_metrics.longest_sleep),
		BK_TICKS_TO_MS(OBK_Mcu_metrics.longest_sleep_req_1s),
		BK_TICKS_TO_MS(OBK_Mcu_metrics.longest_sleep_req),
		OBK_Mcu_metrics.nexttask,
		OBK_Mcu_metrics.task
	);
	ADDLOGF_DEBUG("PS: nosleep reasons %d %d %d %d",
		OBK_Mcu_metrics.reasons[0],
		OBK_Mcu_metrics.reasons[1],
		OBK_Mcu_metrics.reasons[2],
		OBK_Mcu_metrics.reasons[3],
		OBK_Mcu_metrics.reasons[4]
	);
	memset(OBK_Mcu_metrics.reasons, 0, sizeof(OBK_Mcu_metrics.reasons));
	OBK_Mcu_metrics.slept_ticks = 0;
	OBK_Mcu_metrics.sleep_requested_ticks = 0;
	OBK_Mcu_metrics.longest_sleep_1s = 0;
	OBK_Mcu_metrics.longest_sleep_req_1s = 0;
}
#endif
#if ENABLE_HA_DISCOVERY
void Main_ScheduleHomeAssistantDiscovery(int seconds) {
	g_doHomeAssistantDiscoveryIn = seconds;
}
#endif
void Main_ConnectToWiFiNow() {
	const char* wifi_ssid, * wifi_pass;
	g_bOpenAccessPointMode = 0;
	CheckForSSID12_Switch();
	wifi_ssid = CFG_GetWiFiSSIDX();
	wifi_pass = CFG_GetWiFiPassX();
	HAL_WiFi_SetupStatusCallback(Main_OnWiFiStatusChange);
	ADDLOGF_INFO("Registered for wifi changes");
	ADDLOGF_INFO("Connecting to SSID [%s]", wifi_ssid);
	if(CFG_HasFlag(OBK_FLAG_WIFI_ENHANCED_FAST_CONNECT))
	{
		HAL_FastConnectToWiFi(wifi_ssid, &g_cfg.staticIP);
	}
	else
	{
		HAL_ConnectToWiFi(wifi_ssid, wifi_pass, &g_cfg.staticIP);
	}
}
bool Main_HasFastConnect() {
	if (g_bootFailures > 2)
	{
		HAL_DisableEnhancedFastConnect();
		return false;
	}
	if (CFG_HasFlag(OBK_FLAG_WIFI_FAST_CONNECT)) {
		return true;
	}
	if ((PIN_FindPinIndexForRole(IOR_DoorSensorWithDeepSleep, -1) != -1) ||
		(PIN_FindPinIndexForRole(IOR_DoorSensorWithDeepSleep_NoPup, -1) != -1) ||
		(PIN_FindPinIndexForRole(IOR_DoorSensorWithDeepSleep_pd, -1))
	{
		return true;
	}
	return false;
}
#if PLATFORM_LN882H || PLATFORM_ESPIDF || PLATFORM_LN8825
float g_wifi_temperature = 0;
#endif
static byte g_secondsSpentInLowMemoryWarning = 0;
void Main_OnEverySecond()
{
#if PLATFORM_W600 || PLATFORM_W800
#define TimeOut_t xTimeOutType
#endif
#if ! ( WINDOWS || PLATFORM_TXW81X  || PLATFORM_RDA5981)
	TimeOut_t myTimeout;
#endif
	int newMQTTState;
	const char* safe;
	int i;
#ifdef WINDOWS
	g_bHasWiFiConnected = 1;
#endif
	if (!bSafeMode && g_bootFailures <= 1)
	{
#if PLATFORM_BEKEN
		UINT32 temperature;
		temp_single_get_current_temperature(&temperature);
#if PLATFORM_BK7231N
		g_wifi_temperature = (-0.38f * temperature) + 156.0f;
#elif PLATFORM_BK7238 || PLATFORM_BK7252N
		g_wifi_temperature = (-0.4f * temperature) + 131.0f;
#else
		g_wifi_temperature = temperature * 0.128f;
#endif
#elif PLATFORM_BL602 && !PLATFORM_BL602
		get_tsen_adc(&g_wifi_temperature, 0);
#elif PLATFORM_W600 || PLATFORM_W800
		g_wifi_temperature = HAL_ADC_Temp();
#elif PLATFORM_ECR6600
		g_wifi_temperature = hal_adc_tempsensor();
#endif
	}
#if ENABLE_MQTT
	newMQTTState = MQTT_RunEverySecondUpdate();
	if (newMQTTState != bMQTTconnected) {
		bMQTTconnected = newMQTTState;
		if (newMQTTState) {
			EventHandlers_FireEvent(CMD_EVENT_MQTT_STATE, 1);
		}
		else {
			EventHandlers_FireEvent(CMD_EVENT_MQTT_STATE, 0);
		}
	}
#endif
	if (g_newWiFiStatus != g_prevWiFiStatus) {
		g_prevWiFiStatus = g_newWiFiStatus;
		EventHandlers_FireEvent(CMD_EVENT_WIFI_STATUS, g_newWiFiStatus);
	}
	if (bMQTTconnected) {
		i = 0;
	} else {
		i = g_noMQTTTime + 1;
	}
	EventHandlers_ProcessVariableChange_Integer(CMD_EVENT_CHANGE_NOMQTTTIME, g_noMQTTTime, i);
	g_noMQTTTime = i;
#if ENABLE_MQTT
	MQTT_Dedup_Tick();
#endif
#if ENABLE_LED_BASIC
	LED_RunOnEverySecond();
#endif
#ifndef OBK_DISABLE_ALL_DRIVERS
	DRV_OnEverySecond();
#if defined(PLATFORM_BEKEN) || defined(WINDOWS) || defined(PLATFORM_BL602) || defined(PLATFORM_ESPIDF)
	UART_RunEverySecond();
#endif
#endif
	if (OTA_GetProgress() == -1)
	{
		CFG_Save_IfThereArePendingChanges();
	}
#if PLATFORM_BEKEN || PLATFORM_W800
	if (xPortGetFreeHeapSize() < 25 * 1000) {
		g_secondsSpentInLowMemoryWarning++;
		if (g_secondsSpentInLowMemoryWarning > 5) {
			HAL_RebootModule();
		}
	}
	else {
		g_secondsSpentInLowMemoryWarning = 0;
	}
#endif
	if (bSafeMode == 0) {
		const char* ip = HAL_GetMyIPString();
		if (strcpy_safe_checkForChanges(g_currentIPString, ip, sizeof(g_currentIPString))) {
#if ENABLE_MQTT
			if (MQTT_IsReady()) {
				MQTT_DoItemPublish(PUBLISHITEM_SELF_IP);
			}
#endif
			EventHandlers_FireEvent(CMD_EVENT_IPCHANGE, 0);
#if ENABLE_HA_DISCOVERY
			if (CFG_HasFlag(OBK_FLAG_AUTOMAIC_HASS_DISCOVERY)) {
				Main_ScheduleHomeAssistantDiscovery(1);
			}
#endif
		}
	}
	if (g_timeSinceLastPingReply != -1 && g_secondsElapsed > 60)
	{
		g_timeSinceLastPingReply++;
		EventHandlers_ProcessVariableChange_Integer(CMD_EVENT_CHANGE_NOPINGTIME, g_prevTimeSinceLastPingReply, g_timeSinceLastPingReply);
		g_prevTimeSinceLastPingReply = g_timeSinceLastPingReply;
		if (CFG_GetPingDisconnectedSecondsToRestart() > 0 && g_timeSinceLastPingReply >= CFG_GetPingDisconnectedSecondsToRestart())
		{
			if (g_bHasWiFiConnected != 0)
			{
				ADDLOGF_INFO("[Ping watchdog] No ping replies within %i seconds. Will try to reconnect.", g_timeSinceLastPingReply);
				HAL_DisconnectFromWifi();
				g_bHasWiFiConnected = 0;
				g_connectToWiFi = 10;
				g_timeSinceLastPingReply = -1;
			}
		}
	}
	if (bSafeMode == 0)
	{
		for (i = 0; i < PLATFORM_GPIO_MAX; i++)
		{
			if (g_cfg.pins.roles[i] == IOR_ADC)
			{
				int value;
				value = HAL_ADC_Read(i);
				CHANNEL_Set(g_cfg.pins.channels[i], value, CHANNEL_SET_FLAG_SILENT);
			}
		}
	}
	for (i = 0; i < 4; i++) {
		if (scheduledDelay[i] > 0) {
			scheduledDelay[i]--;
			if (scheduledDelay[i] <= 0)
			{
				scheduledDelay[i] = -1;
#ifndef OBK_DISABLE_ALL_DRIVERS
				DRV_StopDriver(scheduledDriverName[i]);
				DRV_StartDriver(scheduledDriverName[i]);
#endif
				scheduledDriverName[i][0] = 0;
			}
		}
	}
#if (WINDOWS || PLATFORM_TXW81X || PLATFORM_RDA5981)
	g_secondsElapsed++;
#elif PLATFORM_ESPIDF
	g_secondsElapsed = (int)(esp_timer_get_time() / 1000000);
#else
	vTaskSetTimeOutState(&myTimeout);
	g_secondsElapsed = (int)((((uint64_t)myTimeout.xOverflowCount << (sizeof(portTickType)*8) | myTimeout.xTimeOnEntering)*portTICK_RATE_MS ) / 1000 );
#endif
	if (bSafeMode) {
		safe = "[SAFE] ";
	}
	else {
		safe = "";
	}
#if ENABLE_MQTT
	ADDLOGF_INFO("%sTime %i, idle %i/s, free %d, MQTT %i(%i), bWifi %i, secondsWithNoPing %i, socks %i/%i %s",
		safe, g_secondsElapsed, idleCount, xPortGetFreeHeapSize(), bMQTTconnected,
		MQTT_GetConnectEvents(),g_bHasWiFiConnected, g_timeSinceLastPingReply, LWIP_GetActiveSockets(), LWIP_GetMaxSockets(),
		g_powersave ? "POWERSAVE" : "");
#else
	ADDLOGF_INFO("%sTime %i, idle %i/s, free %d, bWifi %i, secondsWithNoPing %i, socks %i/%i %s",
		safe, g_secondsElapsed, idleCount, xPortGetFreeHeapSize(),g_bHasWiFiConnected, g_timeSinceLastPingReply, LWIP_GetActiveSockets(), LWIP_GetMaxSockets(),
		g_powersave ? "POWERSAVE" : "");
#endif
	idleCount = 0;
	if (!(g_secondsElapsed % 10))
	{
		HAL_PrintNetworkInfo();
	}
#ifdef PLATFORM_BK7231T
#endif
	int bootCompleteSeconds = CFG_GetBootOkSeconds();
	if (g_secondsElapsed > bootCompleteSeconds && g_bBootMarkedOK == false)
	{
		ADDLOGF_INFO("Boot complete time reached (%i seconds)", bootCompleteSeconds);
		HAL_FlashVars_SaveBootComplete();
		g_bBootMarkedOK = true;
	}
#if ENABLE_HA_DISCOVERY
	if (g_doHomeAssistantDiscoveryIn) {
		if (MQTT_IsReady()) {
			g_doHomeAssistantDiscoveryIn--;
			if (g_doHomeAssistantDiscoveryIn == 0) {
				ADDLOGF_INFO("Will do request HA discovery now.");
				doHomeAssistantDiscovery(0, 0);
			}
			else {
				ADDLOGF_INFO("Will scheduled HA discovery in %i seconds", g_doHomeAssistantDiscoveryIn);
			}
		}
		else {
			ADDLOGF_INFO("HA discovery is scheduled, but MQTT connection is not present yet");
		}
	}
#endif
	if (g_openAP)
	{
		if (g_bHasWiFiConnected)
		{
			HAL_DisconnectFromWifi();
			g_bHasWiFiConnected = 0;
		}
		g_openAP--;
		if (0 == g_openAP)
		{
			HAL_SetupWiFiOpenAccessPoint(CFG_GetDeviceName());
			g_bOpenAccessPointMode = 1;
		}
	}
	if (g_startPingWatchDogAfter) {
		g_startPingWatchDogAfter--;
		if (0 == g_startPingWatchDogAfter)
		{
			const char* pingTargetServer;
			pingTargetServer = CFG_GetPingHost();
			if ((pingTargetServer != NULL) && (strlen(pingTargetServer) > 0))
			{
				g_timeSinceLastPingReply = 0;
#if ENABLE_PING_WATCHDOG
				Main_SetupPingWatchDog(pingTargetServer);
#endif
			}
			else {
				g_timeSinceLastPingReply = -1;
			}
		}
	}
	if (g_saveCfgAfter) {
		g_saveCfgAfter--;
		if (!g_saveCfgAfter) {
			CFG_Save_IfThereArePendingChanges();
		}
	}
	if (g_doUnsafeInitIn) {
		g_doUnsafeInitIn--;
		if (!g_doUnsafeInitIn) {
			ADDLOGF_INFO("Going to call Main_ForceUnsafeInit");
			Main_ForceUnsafeInit();
		}
	}
	if (g_reset) {
		g_reset--;
		if (!g_reset) {
			CFG_Save_IfThereArePendingChanges();
#if ENABLE_BL_SHARED
			if (DRV_IsMeasuringPower())
			{
				BL09XX_SaveEmeteringStatistics();
			}
#endif
			DRV_SavePowerMeterDriverStatistics();
			ADDLOGF_INFO("Rebooting...");
			HAL_DisconnectFromWifi();
			HAL_RebootModule();
		}
		else {
			ADDLOGF_INFO("Module reboot in %i...", g_reset);
		}
	}
#if ENABLE_DRIVER_DHT
	if (g_dhtsCount > 0) {
		if (bSafeMode == 0) {
			DHT_OnEverySecond();
		}
	}
#endif
	HAL_Run_WDT();
	rtos_delay_milliseconds(1);
}
#define WIFI_LED_FAST_BLINK_DURATION 250
#define WIFI_LED_SLOW_BLINK_DURATION 500
static int g_wifiLedToggleTime = 0;
static int g_wifi_ledState = 0;
unsigned int g_timeMs = 0;
static uint32_t g_last_time = 0;
int g_bWantPinDeepSleep;
int g_pinDeepSleepWakeUp = 0;
unsigned int g_deltaTimeMS;
void QuickTick(void* param)
{
	if (g_bWantPinDeepSleep) {
		g_bWantPinDeepSleep = 0;
		HAL_DisconnectFromWifi();
		PINS_BeginDeepSleepWithPinWakeUp(g_pinDeepSleepWakeUp);
		return;
}
#if defined(PLATFORM_BEKEN) && defined(BEKEN_PIN_GPI_INTERRUPTS)
#else
	PIN_ticks(param);
#endif
#if defined(PLATFORM_BEKEN)
	g_timeMs = rtos_get_time();
#elif defined(PLATFORM_ESPIDF)
	g_timeMs = esp_timer_get_time() / 1000;
#else
	g_timeMs += QUICK_TMR_DURATION;
#endif
	g_deltaTimeMS = g_timeMs - g_last_time;
	if (g_deltaTimeMS > 0x4000) {
		g_deltaTimeMS = ((g_timeMs + 0x4000) - (g_last_time + 0x4000));
	}
	g_last_time = g_timeMs;
#if ENABLE_OBK_SCRIPTING
	SVM_RunThreads(g_deltaTimeMS);
#endif
#if ENABLE_OBK_BERRY
	extern void Berry_RunThreads(int deltaMS);
	Berry_RunThreads(g_deltaTimeMS);
#endif
	RepeatingEvents_RunUpdate(g_deltaTimeMS * 0.001f);
#ifndef OBK_DISABLE_ALL_DRIVERS
	DRV_RunQuickTick();
#endif
#ifdef WINDOWS
	NewTuyaMCUSimulator_RunQuickTick(g_deltaTimeMS);
#endif
	CMD_RunUartCmndIfRequired();
#if ENABLE_MQTT
	MQTT_RunQuickTick();
#endif
#if ENABLE_LED_BASIC
	if (CFG_HasFlag(OBK_FLAG_LED_SMOOTH_TRANSITIONS) == true) {
		LED_RunQuickColorLerp(g_deltaTimeMS);
	}
#endif
	if (Main_IsOpenAccessPointMode()) {
		g_wifiLedToggleTime += g_deltaTimeMS;
		if (g_wifiLedToggleTime > WIFI_LED_FAST_BLINK_DURATION) {
			g_wifi_ledState = !g_wifi_ledState;
			g_wifiLedToggleTime = 0;
			PIN_set_wifi_led(g_wifi_ledState);
		}
	}
	else if (Main_IsConnectedToWiFi()) {
		PIN_set_wifi_led(1);
	}
	else {
		g_wifiLedToggleTime += g_deltaTimeMS;
		if (g_wifiLedToggleTime > WIFI_LED_SLOW_BLINK_DURATION) {
			g_wifi_ledState = !g_wifi_ledState;
			g_wifiLedToggleTime = 0;
			PIN_set_wifi_led(g_wifi_ledState);
		}
	}
}
#define QT_STACK_SIZE 2048
#if WINDOWS
#elif PLATFORM_BL602 || PLATFORM_W600 || PLATFORM_W800 || PLATFORM_TR6260 || defined(PLATFORM_REALTEK) || PLATFORM_ECR6600 \
	|| PLATFORM_ESP8266 || PLATFORM_ESPIDF || PLATFORM_XRADIO || PLATFORM_LN882H || PLATFORM_LN8825 \
	|| PLATFORM_BL_NEW || PLATFORM_GD32VW553
void quick_timer_thread(void* param)
{
	while (1) {
		rtos_delay_milliseconds(QUICK_TMR_DURATION);
		QuickTick(0);
	}
}
#elif PLATFORM_TXW81X
void quick_timer_thread(void* param)
{
	while (1) {
		os_task_sleep(QUICK_TMR_DURATION);
		QuickTick(0);
	}
}
#else
beken_timer_t g_quick_timer;
#endif
void QuickTick_StartThread(void)
{
#if WINDOWS
#elif PLATFORM_BL602 || PLATFORM_W600 || PLATFORM_W800 || PLATFORM_TR6260 || defined(PLATFORM_REALTEK) || PLATFORM_ECR6600 \
	|| PLATFORM_ESP8266 || PLATFORM_ESPIDF || PLATFORM_XRADIO || PLATFORM_LN882H || PLATFORM_LN8825 || PLATFORM_BL_NEW || PLATFORM_GD32VW553
	xTaskCreate(quick_timer_thread, "quick", QT_STACK_SIZE, NULL, 15, NULL);
#elif PLATFORM_TXW81X
	os_task_create("quick", quick_timer_thread, NULL, 15, 0, NULL, QT_STACK_SIZE);
#elif PLATFORM_RDA5981
	rda_thread_new("quick", quick_timer_thread, NULL, QT_STACK_SIZE, osPriorityNormal);
#else
	OSStatus result;
	result = rtos_init_timer(&g_quick_timer,
		QUICK_TMR_DURATION,
		QuickTick,
		(void*)0);
	ASSERT(kNoErr == result);
	result = rtos_start_timer(&g_quick_timer);
#endif
}
void app_on_generic_dbl_click(int btnIndex)
{
	if (g_secondsElapsed < 5)
	{
		CFG_SetOpenAccessPoint();
	}
}
int Main_IsOpenAccessPointMode()
{
	return g_bOpenAccessPointMode;
}
int Main_IsConnectedToWiFi()
{
	return g_bHasWiFiConnected;
}
#if PLATFORM_ESPIDF || PLATFORM_BL602 || (PLATFORM_WINDOWS == 0)
inline __attribute__((always_inline))
#endif
void isidle() {
	idleCount++;
#ifdef WFI_FUNC
	if(g_use_wfi) WFI_FUNC();
#endif
}
bool g_unsafeInitDone = false;
void Main_Init_AfterDelay_Unsafe(bool bStartAutoRunScripts) {
#if ENABLE_MQTT
	MQTT_init();
#endif
	CMD_Init_Delayed();
	if (bStartAutoRunScripts) {
#if ENABLE_OBK_SCRIPTING
		SVM_RunStartupCommandAsScript();
#else
		CMD_ExecuteCommand(CFG_GetShortStartupCommand(), COMMAND_FLAG_SOURCE_SCRIPT);
#endif
#if ENABLE_OBK_BERRY
		CMD_ExecuteCommand("berry import autoexec", COMMAND_FLAG_SOURCE_SCRIPT);
#endif
	}
}
void Main_Init_BeforeDelay_Unsafe(bool bAutoRunScripts) {
	g_unsafeInitDone = true;
#ifndef OBK_DISABLE_ALL_DRIVERS
	DRV_Generic_Init();
#endif
#if PLATFORM_BEKEN
	int bk_misc_get_start_type();
	g_rebootReason = bk_misc_get_start_type();
#endif
	RepeatingEvents_Init();
	CFG_ApplyChannelStartValues();
	PIN_AddCommands();
	ADDLOGF_DEBUG("Initialised pins");
#if ENABLE_LITTLEFS
	init_lfs(0);
#endif
	PIN_SetGenericDoubleClickCallback(app_on_generic_dbl_click);
	ADDLOGF_DEBUG("Initialised other callbacks");
	init_rest();
	taslike_commands_init();
#if ENABLE_TEST_COMMANDS
	CMD_InitTestCommands();
#endif
#if ENABLE_LED_BASIC
	NewLED_InitCommands();
#endif
	CMD_InitChannelCommands();
	EventHandlers_Init();
	CMD_Init_Early();
#if PLATFORM_WINDOWS
	CMD_InitSimulatorOnlyCommands();
#else
	HAL_RegisterPlatformSpecificCommands();
#if ENABLE_BT_PROXY
	extern void HAL_BTProxy_RegisterCommands();
	HAL_BTProxy_RegisterCommands();
#endif
#endif
	if (CFG_HasFlag(OBK_FLAG_HTTP_PINMONITOR)) {
		CFG_SetFlag(OBK_FLAG_HTTP_PINMONITOR, false);
	}
}
void Main_Init_Before_Delay()
{
	ADDLOGF_INFO("%s", __func__);
	bk_printf("\r%s\r\n", __func__);
	HAL_FlashVars_IncreaseBootCount();
#if defined(PLATFORM_BEKEN)
	bg_register_irda_check_func(isidle);
#elif PLATFORM_TR6260
	system_register_idle_callback(isidle);
#endif
	g_bootFailures = HAL_FlashVars_GetBootFailures();
	if (g_bootFailures > RESTARTS_REQUIRED_FOR_SAFE_MODE)
	{
		bSafeMode = 1;
		ADDLOGF_INFO("###### safe mode activated - boot failures %d", g_bootFailures);
	}
	CFG_InitAndLoad();
#if ENABLE_LITTLEFS
	LFSAddCmds();
#endif
	if (!bSafeMode)
	{
		Main_Init_BeforeDelay_Unsafe(true);
	}
	ADDLOGF_INFO("%s done", __func__);
	bk_printf("\r%s done\r\n", __func__);
}
void Main_Init_Delay()
{
	ADDLOGF_INFO("%s", __func__);
	bk_printf("\r%s\r\n", __func__);
	extended_app_waiting_for_launch2();
	ADDLOGF_INFO("%s done", __func__);
	bk_printf("\r%s done\r\n", __func__);
	g_StartupDelayOver = 1;
}
void Main_Init_After_Delay()
{
	const char* wifi_ssid, * wifi_pass;
	ADDLOGF_INFO("%s", __func__);
#if ALLOW_SSID2
	Init_WiFiSSIDactual_FromChannelIfSet();
#endif
	wifi_ssid = CFG_GetWiFiSSIDX();
	wifi_pass = CFG_GetWiFiPassX();
	HAL_Configure_WDT();
	if ((*wifi_ssid == 0))
	{
		g_openAP = 5;
	}
	else {
		if (bSafeMode)
		{
			g_openAP = 5;
		}
		else {
			if (Main_HasFastConnect()) {
				Main_ConnectToWiFiNow();
			}
			else {
				g_connectToWiFi = 5;
			}
		}
	}
	ADDLOGF_INFO("Using SSID [%s]", wifi_ssid);
	ADDLOGF_INFO("Using Pass [%s]", wifi_pass);
#if MQTT_USE_TLS
	if (!CFG_GetDisableWebServer() || bSafeMode) {
#endif
		HTTPServer_Start();
		ADDLOGF_DEBUG("Started http tcp server");
#if MQTT_USE_TLS
	}
#endif
	if (!bSafeMode)
	{
#if ENABLE_HA_DISCOVERY
		if (CFG_HasFlag(OBK_FLAG_AUTOMAIC_HASS_DISCOVERY)) {
			Main_ScheduleHomeAssistantDiscovery(1);
		}
#endif
		Main_Init_AfterDelay_Unsafe(true);
#if ENABLE_LITTLEFS && ENABLE_LOG2LFS
	uint8_t g_log2lfs = LOG2LFS_SECONDS(CFG_Get_log2lfs());
	if (g_log2lfs > 0) initLog2LFS();
#endif
	}
	ADDLOGF_INFO("%s done", __func__);
}
void Main_Init()
{
	g_unsafeInitDone = false;
	bk_printf("%s, version %s\r\n", DEVICENAME_PREFIX_FULL, USER_SW_VER);
#if ENABLE_LED_BASIC && WINDOWS
	LED_ResetGlobalVariablesToDefaults();
#endif
#if WINDOWS
	CMD_FreeAllCommands();
#endif
	Main_Init_Before_Delay();
	Main_Init_Delay();
	Main_Init_After_Delay();
}
#if PLATFORM_ESPIDF || PLATFORM_BL602 || (PLATFORM_WINDOWS == 0)
#if PLATFORM_REALTEK_NEW
void __wrap_vApplicationIdleHook(void)
{
	__real_vApplicationIdleHook();
}
#else
void vApplicationIdleHook(void)
{
	isidle();
}
#endif
#endif
