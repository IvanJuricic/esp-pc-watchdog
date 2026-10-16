/*
 * PC Watchdog — ESP32 remote power control and monitoring via Telegram
 *
 * DIAGNOSTIC BUILD
 * ----------------
 * This version logs every network operation in detail. Tokens are masked in
 * log output so the serial log can be shared safely.
 *
 * A /diag command runs every check in sequence and reports the results over
 * Telegram, so faults can be identified without a serial cable attached.
 *
 * Commands
 * --------
 *   /status     full health snapshot fetched from the PC
 *   /ping       quick up or down check only
 *   /wake       send a Wake-on-LAN magic packet
 *   /watchdog   report this device's own condition
 *   /diag       run all connectivity checks and report each result
 *   /shutdown   request a graceful poweroff, requires confirmation
 *   /help       list commands
 */

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

/* ==========================================================================
 * Credentials
 * ========================================================================== */

#define WIFI_SSID      ""
#define WIFI_PASS      ""
#define BOT_TOKEN      ""
#define CHAT_ID        ""

#define SHUTDOWN_TOKEN  ""
#define STATS_TOKEN     ""

/* ==========================================================================
 * Target PC
 * ========================================================================== */

#define PC_MAC         {0x10,0xff,0xe0,0x74,0x77,0x6e}
#define PC_IP          "192.168.0.224"
#define BROADCAST_IP   "192.168.0.255"
#define WOL_PORT       9
#define PC_HTTP_PORT   9999

/* ==========================================================================
 * Tunables
 * ========================================================================== */

#define TG_LONGPOLL_S            50
#define HTTP_TIMEOUT_MS          60000
#define STATS_TIMEOUT_MS         15000
#define WDT_TIMEOUT_S            120
#define POLL_INTERVAL_MS         500
#define MAX_CONSECUTIVE_FAILS    20
#define WIFI_WAIT_MS             15000
#define WIFI_RETRY_MAX_MS        30000
#define CONFIRM_WINDOW_US        (60LL * 1000000LL)

#define WIFI_CONNECTED_BIT       BIT0

/* ==========================================================================
 * State
 * ========================================================================== */

static const char *TAG = "watchdog";

static EventGroupHandle_t wifi_events;
static long     last_update_id        = 0;
static int      wifi_retry_delay_ms   = 1000;
static int      consecutive_fails     = 0;
static int64_t  shutdown_requested_at = 0;

/* Records of the most recent request outcomes, so /diag can report what
 * happened without needing a serial connection. */
static int  last_http_status = 0;
static char last_http_err[48] = "none";
static int  last_resp_len = 0;

static char http_buf[6144];
static int  http_len = 0;
static char stats_buf[4096];
static char msg_buf[1600];
static char enc_buf[4096];
static char url_buf[4608];
static char post_buf[5120];
static char diag_buf[1200];

/* ==========================================================================
 * Logging helpers
 * ========================================================================== */

/*
 * Renders a token as first four characters, ellipsis, last four, plus its
 * length.
 *
 * Length is included because the most common token fault is a truncated or
 * whitespace-padded value, which looks identical to a correct one when only
 * the ends are shown.
 *
 * Not reentrant: two calls in one printf will both show the second value.
 */
static const char *mask(const char *tok)
{
    static char out[32];
    size_t n = strlen(tok);

    if (n == 0) {
        snprintf(out, sizeof(out), "EMPTY(0)");
    } else if (n <= 8) {
        snprintf(out, sizeof(out), "SHORT(%u)", (unsigned)n);
    } else {
        snprintf(out, sizeof(out), "%.4s...%s(%u)",
                 tok, tok + n - 4, (unsigned)n);
    }
    return out;
}

/* ==========================================================================
 * WiFi
 * ========================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data;
        xEventGroupClearBits(wifi_events, WIFI_CONNECTED_BIT);

        ESP_LOGW(TAG, "WiFi disconnected (reason %d), retrying in %d ms",
                 e->reason, wifi_retry_delay_ms);

        vTaskDelay(pdMS_TO_TICKS(wifi_retry_delay_ms));
        esp_wifi_connect();

        wifi_retry_delay_ms *= 2;
        if (wifi_retry_delay_ms > WIFI_RETRY_MAX_MS)
            wifi_retry_delay_ms = WIFI_RETRY_MAX_MS;

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;

        /* The gateway and netmask are logged because the PC address is
         * hardcoded: if the ESP32 lands on a different subnet than the PC,
         * every request to it fails with a connect error that otherwise
         * looks like the agent being down. */
        ESP_LOGI(TAG, "Got IP: " IPSTR "  gw: " IPSTR "  mask: " IPSTR,
                 IP2STR(&e->ip_info.ip),
                 IP2STR(&e->ip_info.gw),
                 IP2STR(&e->ip_info.netmask));

        wifi_retry_delay_ms = 1000;
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init(void)
{
    wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid) - 1);
    strncpy((char *)wc.sta.password, WIFI_PASS, sizeof(wc.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi starting, SSID %s", WIFI_SSID);
}

static bool wifi_wait_connected(uint32_t timeout_ms)
{
    EventBits_t bits = xEventGroupWaitBits(
        wifi_events, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
        pdMS_TO_TICKS(timeout_ms));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/* ==========================================================================
 * Diagnostics
 * ========================================================================== */

static const char *reset_reason_str(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:   return "power on";
        case ESP_RST_EXT:       return "external reset";
        case ESP_RST_SW:        return "software restart";
        case ESP_RST_PANIC:     return "panic";
        case ESP_RST_INT_WDT:   return "interrupt watchdog";
        case ESP_RST_TASK_WDT:  return "task watchdog";
        case ESP_RST_WDT:       return "watchdog";
        case ESP_RST_BROWNOUT:  return "brownout";
        case ESP_RST_DEEPSLEEP: return "deep sleep wake";
        default:                return "unknown";
    }
}

static void format_duration(long seconds, char *out, int cap)
{
    long d = seconds / 86400;
    long h = (seconds % 86400) / 3600;
    long m = (seconds % 3600) / 60;

    if (d > 0)      snprintf(out, cap, "%ldd %ldh %ldm", d, h, m);
    else if (h > 0) snprintf(out, cap, "%ldh %ldm", h, m);
    else            snprintf(out, cap, "%ldm", m);
}

/* ==========================================================================
 * Wake-on-LAN
 * ========================================================================== */

static bool send_wol(void)
{
    uint8_t mac[6] = PC_MAC;
    uint8_t packet[102];

    memset(packet, 0xFF, 6);
    for (int i = 1; i <= 16; i++)
        memcpy(&packet[i * 6], mac, 6);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "WOL: socket() failed, errno=%d", errno);
        return false;
    }

    int broadcast = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                   &broadcast, sizeof(broadcast)) < 0) {
        ESP_LOGW(TAG, "WOL: SO_BROADCAST failed, errno=%d", errno);
    }

    struct sockaddr_in addr = { 0 };
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(WOL_PORT);
    addr.sin_addr.s_addr = inet_addr(BROADCAST_IP);

    int sent = sendto(sock, packet, sizeof(packet), 0,
                      (struct sockaddr *)&addr, sizeof(addr));
    int e = errno;
    close(sock);

    if (sent != sizeof(packet)) {
        ESP_LOGW(TAG, "WOL: sendto returned %d, errno=%d", sent, e);
        return false;
    }

    ESP_LOGI(TAG, "WOL: sent %d bytes to %s:%d", sent, BROADCAST_IP, WOL_PORT);
    return true;
}

/* ==========================================================================
 * PC interaction
 * ========================================================================== */

/*
 * Opens a TCP connection to the given port on the PC.
 *
 * errno is captured immediately after connect and before close, because
 * close overwrites it. The distinction matters: ECONNREFUSED means the host
 * is reachable but nothing is listening, while EHOSTUNREACH or a timeout
 * means the host itself cannot be reached.
 */
static bool tcp_probe(int port, int timeout_s)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        ESP_LOGE(TAG, "probe %d: socket() failed errno=%d", port, errno);
        return false;
    }

    struct timeval tv = { .tv_sec = timeout_s, .tv_usec = 0 };
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr = { 0 };
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(port);
    addr.sin_addr.s_addr = inet_addr(PC_IP);

    int rc = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    int e = errno;
    close(sock);

    if (rc == 0) {
        ESP_LOGI(TAG, "probe %s:%d OK", PC_IP, port);
        return true;
    }

    ESP_LOGW(TAG, "probe %s:%d FAILED rc=%d errno=%d (%s)",
             PC_IP, port, rc, e,
             e == ECONNREFUSED ? "nothing listening" :
             e == EHOSTUNREACH ? "host unreachable" :
             e == ETIMEDOUT    ? "timed out" : "other");
    return false;
}

static bool pc_is_up(void)
{
    return tcp_probe(22, 2);
}

/* ==========================================================================
 * HTTP
 * ========================================================================== */

static char *ev_target = NULL;
static int   ev_cap    = 0;

static esp_err_t http_ev(esp_http_client_event_t *e)
{
    switch (e->event_id) {
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "http: connected");
        break;
    case HTTP_EVENT_ON_DATA:
        if (e->data_len > 0 && ev_target) {
            if (http_len + e->data_len < ev_cap) {
                memcpy(ev_target + http_len, e->data, e->data_len);
                http_len += e->data_len;
            } else {
                ESP_LOGW(TAG, "http: response exceeds %d bytes, truncated",
                         ev_cap);
            }
        }
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "http: disconnected");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/*
 * Performs a GET into the supplied buffer. Returns true only on HTTP 200.
 *
 * The label argument names the call site so interleaved log lines can be
 * attributed. Outcomes are also recorded in the last_http_* globals for the
 * /diag command to report.
 */
static bool http_get(const char *label, const char *url, char *dest, int cap,
                     bool tls, int timeout_ms)
{
    http_len  = 0;
    ev_target = dest;
    ev_cap    = cap;
    dest[0]   = '\0';

    ESP_LOGI(TAG, "[%s] GET start, tls=%d timeout=%dms", label, tls, timeout_ms);

    esp_http_client_config_t cfg = {
        .url            = url,
        .event_handler  = http_ev,
        .timeout_ms     = timeout_ms,
        .buffer_size    = 2048,
        .buffer_size_tx = 1024,
    };
    if (tls) cfg.crt_bundle_attach = esp_crt_bundle_attach;

    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        ESP_LOGE(TAG, "[%s] client_init failed", label);
        snprintf(last_http_err, sizeof(last_http_err), "init failed");
        ev_target = NULL;
        return false;
    }

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_perform(c);
    int elapsed_ms = (int)((esp_timer_get_time() - t0) / 1000);

    int code = esp_http_client_get_status_code(c);
    int64_t clen = esp_http_client_get_content_length(c);

    last_http_status = code;
    last_resp_len    = http_len;
    snprintf(last_http_err, sizeof(last_http_err), "%s", esp_err_to_name(err));

    ESP_LOGI(TAG, "[%s] err=%s code=%d content_len=%lld got=%d bytes in %dms",
             label, esp_err_to_name(err), code, clen, http_len, elapsed_ms);

    bool ok = (err == ESP_OK && code == 200);

    if (!ok) {
        /* 403 from the agent means the wrong token for that endpoint.
         * 401 from Telegram means the bot token is rejected.
         * 409 from Telegram means a webhook is set or another client is
         * polling the same token.
         * A connect error means the request never reached the server. */
        ESP_LOGW(TAG, "[%s] FAILED", label);
        if (http_len > 0) {
            dest[http_len] = '\0';
            ESP_LOGW(TAG, "[%s] body: %.200s", label, dest);
        }
    }

    esp_http_client_cleanup(c);
    dest[http_len] = '\0';
    ev_target = NULL;
    return ok;
}

static bool http_post_form(const char *label, const char *url, const char *body)
{
    http_len    = 0;
    ev_target   = http_buf;
    ev_cap      = sizeof(http_buf);
    http_buf[0] = '\0';

    ESP_LOGI(TAG, "[%s] POST start, body=%d bytes", label, (int)strlen(body));

    esp_http_client_config_t cfg = {
        .url               = url,
        .method            = HTTP_METHOD_POST,
        .event_handler     = http_ev,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms        = HTTP_TIMEOUT_MS,
        .buffer_size       = 2048,
        .buffer_size_tx    = 2048,
    };

    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) {
        ESP_LOGE(TAG, "[%s] client_init failed", label);
        ev_target = NULL;
        return false;
    }

    esp_http_client_set_header(c, "Content-Type",
                               "application/x-www-form-urlencoded");
    esp_http_client_set_post_field(c, body, strlen(body));

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_perform(c);
    int elapsed_ms = (int)((esp_timer_get_time() - t0) / 1000);
    int code = esp_http_client_get_status_code(c);

    ESP_LOGI(TAG, "[%s] err=%s code=%d in %dms",
             label, esp_err_to_name(err), code, elapsed_ms);

    bool ok = (err == ESP_OK && code == 200);
    if (!ok && http_len > 0) {
        http_buf[http_len] = '\0';
        ESP_LOGW(TAG, "[%s] body: %.200s", label, http_buf);
    }

    esp_http_client_cleanup(c);
    ev_target = NULL;
    return ok;
}

static bool send_shutdown(void)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d/shutdown/%s",
             PC_IP, PC_HTTP_PORT, SHUTDOWN_TOKEN);

    ESP_LOGI(TAG, "shutdown: target %s:%d token %s",
             PC_IP, PC_HTTP_PORT, mask(SHUTDOWN_TOKEN));

    /* A 200 may still be reported as an error: the PC begins powering off
     * while the response is in flight, so the connection can drop before it
     * is fully read. Check whether the machine actually went down rather
     * than trusting this return value alone. */
    return http_get("shutdown", url, http_buf, sizeof(http_buf), false, 5000);
}

static void url_encode(const char *in, char *out, int cap)
{
    static const char hex[] = "0123456789ABCDEF";
    int j = 0;

    for (int i = 0; in[i] && j < cap - 4; i++) {
        unsigned char c = (unsigned char)in[i];
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') ||
            c == '-' || c == '_' || c == '.' || c == '~') {
            out[j++] = (char)c;
        } else {
            out[j++] = '%';
            out[j++] = hex[c >> 4];
            out[j++] = hex[c & 0x0F];
        }
    }
    out[j] = '\0';
}

static void tg_send(const char *text)
{
    url_encode(text, enc_buf, sizeof(enc_buf));
    snprintf(url_buf, sizeof(url_buf),
             "https://api.telegram.org/bot%s/sendMessage", BOT_TOKEN);
    snprintf(post_buf, sizeof(post_buf),
             "chat_id=%s&text=%s", CHAT_ID, enc_buf);
    http_post_form("tg_send", url_buf, post_buf);
}

static void tg_set_commands(void)
{
    const char *json =
        "["
        "{\"command\":\"status\",\"description\":\"Full PC health report\"},"
        "{\"command\":\"ping\",\"description\":\"Quick up or down check\"},"
        "{\"command\":\"wake\",\"description\":\"Power on the PC\"},"
        "{\"command\":\"watchdog\",\"description\":\"ESP32 device health\"},"
        "{\"command\":\"diag\",\"description\":\"Run connectivity checks\"},"
        "{\"command\":\"help\",\"description\":\"Show commands\"},"
        "{\"command\":\"shutdown\",\"description\":\"Power off the PC\"}"
        "]";

    url_encode(json, enc_buf, sizeof(enc_buf));
    snprintf(url_buf, sizeof(url_buf),
             "https://api.telegram.org/bot%s/setMyCommands", BOT_TOKEN);
    snprintf(post_buf, sizeof(post_buf), "commands=%s", enc_buf);
    http_post_form("set_commands", url_buf, post_buf);
}

/* ==========================================================================
 * Stats formatting
 * ========================================================================== */

static double json_num(cJSON *obj, const char *key, bool *found)
{
    cJSON *item = cJSON_GetObjectItem(obj, key);
    if (item && cJSON_IsNumber(item)) {
        if (found) *found = true;
        return item->valuedouble;
    }
    if (found) *found = false;
    return 0;
}

#define APPEND(...) do { \
    int _n = snprintf(msg_buf + pos, sizeof(msg_buf) - pos, __VA_ARGS__); \
    if (_n > 0 && pos + _n < (int)sizeof(msg_buf)) pos += _n; \
} while (0)

/*
 * Fetches the PC's stats document and renders it.
 *
 * Returns a distinct code so the caller can tell the failure modes apart,
 * rather than reporting "agent not running" for every one of them:
 *   0  success
 *   1  HTTP request failed (unreachable, wrong token, agent down)
 *   2  request succeeded but the body did not parse as JSON
 */
static int build_status_message(void)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%d/stats/%s",
             PC_IP, PC_HTTP_PORT, STATS_TOKEN);

    ESP_LOGI(TAG, "stats: target %s:%d token %s",
             PC_IP, PC_HTTP_PORT, mask(STATS_TOKEN));

    if (!http_get("stats", url, stats_buf, sizeof(stats_buf),
                  false, STATS_TIMEOUT_MS))
        return 1;

    ESP_LOGI(TAG, "stats: received %d bytes", last_resp_len);
    ESP_LOGI(TAG, "stats: body starts: %.120s", stats_buf);

    cJSON *root = cJSON_Parse(stats_buf);
    if (!root) {
        /* cJSON reports where parsing stopped, which distinguishes a
         * truncated response from a malformed one. */
        const char *errptr = cJSON_GetErrorPtr();
        ESP_LOGW(TAG, "stats: JSON parse failed at offset %d",
                 errptr ? (int)(errptr - stats_buf) : -1);
        return 2;
    }

    int  pos = 0;
    bool found;
    char tmp[64];

    APPEND("PC STATUS\n");
    APPEND("=====================\n\n");

    double up = json_num(root, "uptime", &found);
    if (found) {
        format_duration((long)up, tmp, sizeof(tmp));
        APPEND("Uptime:  %s\n\n", tmp);
    }

    APPEND("TEMPERATURES\n");
    double cpu_t = json_num(root, "cpu_temp", &found);
    if (found) APPEND("  CPU:   %.1f C\n", cpu_t);

    cJSON *gpu = cJSON_GetObjectItem(root, "gpu");
    if (gpu && cJSON_IsObject(gpu)) {
        double gt = json_num(gpu, "temp", &found);
        if (found) APPEND("  GPU:   %.0f C\n", gt);
    }

    double nvme_t = json_num(root, "nvme_temp", &found);
    if (found) APPEND("  NVMe:  %.0f C\n", nvme_t);
    APPEND("\n");

    APPEND("LOAD AND MEMORY\n");
    cJSON *load = cJSON_GetObjectItem(root, "load");
    if (load && cJSON_IsArray(load) && cJSON_GetArraySize(load) >= 3) {
        APPEND("  Load:  %.2f  %.2f  %.2f\n",
               cJSON_GetArrayItem(load, 0)->valuedouble,
               cJSON_GetArrayItem(load, 1)->valuedouble,
               cJSON_GetArrayItem(load, 2)->valuedouble);
    }

    cJSON *mem = cJSON_GetObjectItem(root, "memory");
    if (mem && cJSON_IsObject(mem)) {
        double total = json_num(mem, "total", &found);
        double avail = json_num(mem, "available", NULL);
        if (found && total > 0) {
            double used_gb = (total - avail) / 1073741824.0;
            double tot_gb  = total / 1073741824.0;
            APPEND("  RAM:   %.1f / %.0f GB  (%.0f%%)\n",
                   used_gb, tot_gb, (used_gb / tot_gb) * 100.0);
        }
    }
    APPEND("\n");

    if (gpu && cJSON_IsObject(gpu)) {
        APPEND("GPU\n");
        double v;

        v = json_num(gpu, "util", &found);
        if (found) APPEND("  Usage: %.0f%%\n", v);

        double vram_u = json_num(gpu, "mem_used", &found);
        double vram_t = json_num(gpu, "mem_total", NULL);
        if (found && vram_t > 0)
            APPEND("  VRAM:  %.1f / %.1f GB\n",
                   vram_u / 1024.0, vram_t / 1024.0);

        v = json_num(gpu, "power", &found);
        if (found) APPEND("  Power: %.0f W\n", v);

        v = json_num(gpu, "fan", &found);
        if (found) APPEND("  Fan:   %.0f%%\n", v);
        APPEND("\n");
    }

    cJSON *disk = cJSON_GetObjectItem(root, "disk");
    cJSON *nvme = cJSON_GetObjectItem(root, "nvme");
    if ((disk && cJSON_IsObject(disk)) || (nvme && cJSON_IsObject(nvme))) {
        APPEND("STORAGE\n");

        if (disk && cJSON_IsObject(disk)) {
            double free_b  = json_num(disk, "free", &found);
            double total_b = json_num(disk, "total", NULL);
            if (found && total_b > 0)
                APPEND("  Free:  %.0f GB of %.0f GB\n",
                       free_b / 1073741824.0, total_b / 1073741824.0);
        }

        if (nvme && cJSON_IsObject(nvme)) {
            double wear = json_num(nvme, "percent_used", &found);
            if (found) APPEND("  Wear:  %.0f%%\n", wear);

            double errs = json_num(nvme, "media_errors", &found);
            if (found) APPEND("  Errors: %.0f\n", errs);

            double unsafe = json_num(nvme, "unsafe_shutdowns", &found);
            if (found) APPEND("  Unsafe stops: %.0f\n", unsafe);
        }
        APPEND("\n");
    }

    APPEND("SERVICES\n");

    cJSON *failed = cJSON_GetObjectItem(root, "failed_units");
    if (failed && cJSON_IsArray(failed)) {
        int n = cJSON_GetArraySize(failed);
        if (n == 0) {
            APPEND("  Units: all OK\n");
        } else {
            APPEND("  Units: %d FAILED\n", n);
            for (int i = 0; i < n && i < 4; i++) {
                cJSON *u = cJSON_GetArrayItem(failed, i);
                if (cJSON_IsString(u)) APPEND("    - %s\n", u->valuestring);
            }
        }
    }

    cJSON *cont = cJSON_GetObjectItem(root, "containers");
    if (cont && cJSON_IsObject(cont)) {
        double n = json_num(cont, "running", &found);
        if (found) APPEND("  Podman: %.0f running\n", n);
    }

    cJSON *disp = cJSON_GetObjectItem(root, "display");
    if (disp && cJSON_IsBool(disp)) {
        APPEND("  Display: %s\n",
               cJSON_IsTrue(disp) ? "connected" : "none detected");
    }

    cJSON_Delete(root);
    msg_buf[pos] = '\0';
    ESP_LOGI(TAG, "stats: built message of %d chars", pos);
    return 0;
}

/* ==========================================================================
 * /diag
 * ========================================================================== */

/*
 * Runs every check in sequence and reports each result.
 *
 * Exists so faults can be located without a serial cable, which matters
 * because the most likely time to need it is when away from the machine.
 * Each step is reported separately so the first failing layer is visible
 * rather than only the final symptom.
 */
static void run_diagnostics(void)
{
    int pos = 0;

    #define DAPPEND(...) do { \
        int _n = snprintf(diag_buf + pos, sizeof(diag_buf) - pos, __VA_ARGS__); \
        if (_n > 0 && pos + _n < (int)sizeof(diag_buf)) pos += _n; \
    } while (0)

    DAPPEND("DIAGNOSTICS\n");
    DAPPEND("=====================\n\n");

    /* ---- 1. this device ---- */
    esp_netif_ip_info_t ip;
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        DAPPEND("ESP32 IP:  " IPSTR "\n", IP2STR(&ip.ip));
        DAPPEND("Gateway:   " IPSTR "\n", IP2STR(&ip.gw));
    } else {
        DAPPEND("ESP32 IP:  unknown\n");
    }
    DAPPEND("Heap free: %lu B\n\n", (unsigned long)esp_get_free_heap_size());

    /* ---- 2. reachability ---- */
    DAPPEND("PC target: %s\n", PC_IP);

    bool ssh = tcp_probe(22, 3);
    DAPPEND("  Port 22 (SSH):   %s\n", ssh ? "open" : "no answer");

    bool agent = tcp_probe(PC_HTTP_PORT, 3);
    DAPPEND("  Port 9999 (agent): %s\n\n",
            agent ? "open" : "no answer");

    if (!ssh && !agent) {
        DAPPEND("PC unreachable. Either it is off, or its\n");
        DAPPEND("address changed from %s.\n", PC_IP);
        tg_send(diag_buf);
        return;
    }

    if (ssh && !agent) {
        DAPPEND("PC is up but the agent is not listening.\n");
        DAPPEND("Check on the PC:\n");
        DAPPEND("  systemctl --user status pc-agent\n");
        tg_send(diag_buf);
        return;
    }

    /* ---- 3. stats endpoint ---- */
    DAPPEND("Stats token: %s\n", mask(STATS_TOKEN));

    int rc = build_status_message();
    if (rc == 0) {
        DAPPEND("  /status: OK (%d bytes)\n\n", last_resp_len);
    } else if (rc == 1) {
        DAPPEND("  /status: HTTP %d (%s)\n", last_http_status, last_http_err);
        if (last_http_status == 403)
            DAPPEND("  403 means the stats token does not\n"
                    "  match pc-agent.env.\n\n");
        else
            DAPPEND("\n");
    } else {
        DAPPEND("  /status: bad JSON (%d bytes)\n\n", last_resp_len);
    }

    /* ---- 4. shutdown token, without shutting down ----
     *
     * The shutdown endpoint cannot be tested directly without powering the
     * machine off, so the token is only reported for comparison. A stats
     * failure with 403 and a shutdown failure with 403 have the same cause
     * and the same fix. */
    DAPPEND("Shutdown token: %s\n", mask(SHUTDOWN_TOKEN));
    DAPPEND("  (not tested: would power off the PC)\n");

    tg_send(diag_buf);
    #undef DAPPEND
}

/* ==========================================================================
 * Command handling
 * ========================================================================== */

static void handle_cmd(const char *text, const char *from_id)
{
    if (strcmp(from_id, CHAT_ID) != 0) {
        ESP_LOGW(TAG, "ignoring command from unauthorised id %s", from_id);
        return;
    }

    char cmd[32] = { 0 };
    int i = 0;
    while (text[i] && text[i] != ' ' && text[i] != '@' && i < 31) {
        cmd[i] = text[i];
        i++;
    }

    ESP_LOGI(TAG, "=== command received: %s ===", cmd);

    int64_t now = esp_timer_get_time();

    if (strcmp(cmd, "/confirm") == 0) {
        if (shutdown_requested_at == 0) {
            tg_send("Nothing to confirm.");
        } else if (now - shutdown_requested_at > CONFIRM_WINDOW_US) {
            shutdown_requested_at = 0;
            tg_send("Confirmation expired. Send /shutdown again.");
        } else {
            shutdown_requested_at = 0;
            if (send_shutdown()) {
                tg_send("Shutdown command sent. PC is powering off.");
            } else {
                snprintf(msg_buf, sizeof(msg_buf),
                         "Shutdown request failed.\n\n"
                         "HTTP status: %d\n"
                         "Error: %s\n\n"
                         "403 means the shutdown token does not match\n"
                         "the one in pc-agent.env.\n"
                         "Send /diag for more detail.",
                         last_http_status, last_http_err);
                tg_send(msg_buf);
            }
        }
        return;
    }

    shutdown_requested_at = 0;

    if (strcmp(cmd, "/status") == 0) {
        if (!pc_is_up()) {
            tg_send("PC STATUS\n"
                    "=====================\n\n"
                    "PC is OFF\n\n"
                    "Send /wake to power it on.");
            return;
        }

        int rc = build_status_message();
        if (rc == 0) {
            tg_send(msg_buf);
        } else {
            /* Reported separately rather than as one generic message,
             * because a token mismatch and a stopped service need
             * different fixes. */
            snprintf(msg_buf, sizeof(msg_buf),
                     "Could not read stats.\n\n"
                     "HTTP status: %d\n"
                     "Error: %s\n"
                     "Bytes received: %d\n\n"
                     "%s\n\n"
                     "Send /diag for a full check.",
                     last_http_status, last_http_err, last_resp_len,
                     last_http_status == 403
                        ? "403: the stats token does not match pc-agent.env."
                        : rc == 2
                        ? "The response was not valid JSON."
                        : "The agent did not respond on port 9999.");
            tg_send(msg_buf);
        }

    } else if (strcmp(cmd, "/ping") == 0) {
        tg_send(pc_is_up() ? "PC is UP  (SSH responding)" : "PC is DOWN");

    } else if (strcmp(cmd, "/diag") == 0) {
        run_diagnostics();

    } else if (strcmp(cmd, "/wake") == 0) {
        if (pc_is_up()) {
            tg_send("PC is already running.");
        } else if (send_wol()) {
            tg_send("Wake-on-LAN packet sent.\n\n"
                    "Boot takes about a minute.\n"
                    "Send /ping to check.");
        } else {
            tg_send("Failed: could not send the wake packet.");
        }

    } else if (strcmp(cmd, "/shutdown") == 0) {
        if (!pc_is_up()) {
            tg_send("PC is already off.");
        } else {
            shutdown_requested_at = now;
            tg_send("CONFIRM SHUTDOWN\n"
                    "=====================\n\n"
                    "This will power off the PC.\n\n"
                    "Send /confirm within 60 seconds.\n"
                    "Any other command cancels.");
        }

    } else if (strcmp(cmd, "/watchdog") == 0) {
        char up[64];
        format_duration((long)(esp_timer_get_time() / 1000000LL),
                        up, sizeof(up));

        snprintf(msg_buf, sizeof(msg_buf),
                 "WATCHDOG DEVICE\n"
                 "=====================\n\n"
                 "Uptime:       %s\n"
                 "Free heap:    %lu B\n"
                 "Min heap:     %lu B\n"
                 "Failed polls: %d\n"
                 "Last reset:   %s\n"
                 "Last HTTP:    %d (%s)\n",
                 up,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 consecutive_fails,
                 reset_reason_str(),
                 last_http_status, last_http_err);
        tg_send(msg_buf);

    } else if (strcmp(cmd, "/help") == 0 || strcmp(cmd, "/start") == 0) {
        tg_send("PC WATCHDOG\n"
                "=====================\n\n"
                "/status\n"
                "   Full health report\n\n"
                "/ping\n"
                "   Quick up or down check\n\n"
                "/wake\n"
                "   Power on the PC\n\n"
                "/watchdog\n"
                "   This device's own health\n\n"
                "/diag\n"
                "   Run connectivity checks\n\n"
                "/help\n"
                "   Show this list\n\n"
                "- - - - - - - - - - -\n\n"
                "/shutdown\n"
                "   Power off the PC\n"
                "   (asks for confirmation)");

    } else {
        ESP_LOGI(TAG, "unrecognised command: %s", cmd);
    }

    ESP_LOGI(TAG, "=== command %s done ===", cmd);
}

/* ==========================================================================
 * Polling
 * ========================================================================== */

static bool poll_telegram(void)
{
    snprintf(url_buf, sizeof(url_buf),
             "https://api.telegram.org/bot%s/getUpdates?offset=%ld&timeout=%d",
             BOT_TOKEN, last_update_id + 1, TG_LONGPOLL_S);

    if (!http_get("poll", url_buf, http_buf, sizeof(http_buf),
                  true, HTTP_TIMEOUT_MS))
        return false;

    cJSON *root = cJSON_Parse(http_buf);
    if (!root) {
        ESP_LOGW(TAG, "poll: JSON parse failed, %d bytes: %.120s",
                 http_len, http_buf);
        return false;
    }

    cJSON *result = cJSON_GetObjectItem(root, "result");
    if (cJSON_IsArray(result)) {
        int n = cJSON_GetArraySize(result);
        if (n > 0) ESP_LOGI(TAG, "poll: %d update(s)", n);

        cJSON *upd;
        cJSON_ArrayForEach(upd, result) {
            cJSON *uid = cJSON_GetObjectItem(upd, "update_id");
            if (uid) last_update_id = (long)uid->valuedouble;

            cJSON *msg = cJSON_GetObjectItem(upd, "message");
            if (!msg) continue;

            cJSON *text = cJSON_GetObjectItem(msg, "text");
            cJSON *from = cJSON_GetObjectItem(msg, "from");
            cJSON *fid  = from ? cJSON_GetObjectItem(from, "id") : NULL;

            if (text && cJSON_IsString(text) && fid) {
                char idbuf[32];
                snprintf(idbuf, sizeof(idbuf), "%.0f", fid->valuedouble);
                handle_cmd(text->valuestring, idbuf);
            }
        }
    }

    cJSON_Delete(root);
    return true;
}

/* ==========================================================================
 * Main task
 * ========================================================================== */

static void watchdog_task(void *arg)
{
    esp_task_wdt_config_t wdt_cfg = {
        .timeout_ms     = WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,
        .trigger_panic  = true,
    };

    esp_err_t err = esp_task_wdt_init(&wdt_cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_cfg));
    } else {
        ESP_ERROR_CHECK(err);
    }
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    /* Token lengths are logged at startup because a truncated or padded
     * token is the single most common configuration fault, and it is
     * invisible from the values themselves. */
    ESP_LOGI(TAG, "config: bot=%s chat=%s",
             mask(BOT_TOKEN), CHAT_ID);
    ESP_LOGI(TAG, "config: stats token %s", mask(STATS_TOKEN));
    ESP_LOGI(TAG, "config: shutdown token %s", mask(SHUTDOWN_TOKEN));
    ESP_LOGI(TAG, "config: PC %s, agent port %d", PC_IP, PC_HTTP_PORT);

    if (wifi_wait_connected(30000)) {
        tg_set_commands();

        char note[160];
        snprintf(note, sizeof(note),
                 "Watchdog online.\nLast reset: %s\n\n"
                 "Send /help for commands,\n"
                 "or /diag to check connectivity.",
                 reset_reason_str());
        tg_send(note);
    } else {
        ESP_LOGW(TAG, "no WiFi at startup, entering poll loop anyway");
    }

    while (1) {
        esp_task_wdt_reset();

        if (!wifi_wait_connected(WIFI_WAIT_MS)) {
            ESP_LOGW(TAG, "WiFi unavailable, skipping poll");
            consecutive_fails++;
        } else if (poll_telegram()) {
            consecutive_fails = 0;
        } else {
            consecutive_fails++;
            ESP_LOGW(TAG, "poll failed (%d consecutive)", consecutive_fails);
        }

        if (consecutive_fails >= MAX_CONSECUTIVE_FAILS) {
            ESP_LOGE(TAG, "%d consecutive failures, restarting",
                     consecutive_fails);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_restart();
        }

        ESP_LOGI(TAG, "cycle done, fails=%d heap=%lu min=%lu",
                 consecutive_fails,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "boot, reset reason: %s", reset_reason_str());

    wifi_init();
    xTaskCreate(watchdog_task, "watchdog", 12288, NULL, 5, NULL);
}