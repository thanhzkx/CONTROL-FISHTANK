#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_log.h"

#define DS18B20_GPIO 4  // Chân GPIO kết nối với DS18B20
#define RELAY_GPIO 16 // GPIO để điều khiển rơ le
#define BUTTON_MODE_GPIO 15  // Chân nút nhấn chuyển chế độ
#define BUTTON_RELAY_GPIO 14  // Chân GPIO cho nút nhấn điều khiển relay

#define I2C_SDA 21
#define I2C_SCL 22
#define LCD_ADDR 0x27  // Dia chi I2C cua LCD, neu khong hoat dong thi 0x3F
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define DHT11_GPIO 17 // Chân GPIO kết nối DHT11

#define TEMP_THRESHOLD_1 24.0   // Ngưỡng 1
#define TEMP_THRESHOLD_2 28.0   // Ngưỡng 2
#define TEMP_THRESHOLD_3 20.0   // Ngưỡng 3
#define TEMP_THRESHOLD_4 35.0   // Ngưỡng 4
#define MENU_BUTTON_GPIO 5  // Chân GPIO để sử dụng làm nút menu
#define ALERT_LED_GPIO 2
#define BUTTON_UP_GPIO 12 // Thay đổi theo GPIO bạn sử dụng cho nút UP
#define BUTTON_DOWN_GPIO 13 // Thay đổi theo GPIO bạn sử dụng cho nút DOWN
bool in_menu = false;
int64_t last_menu_interaction_time = 0;
         

// Biến trạng thái cho menu
int menu_state = 0;             // Trạng thái menu (1 đến 5)

// Biến trạng thái
bool auto_mode = true; // Bật chế độ tự động ban đầu

float TEMP_LOW_THRESHOLD = 24.0;
float TEMP_HIGH_THRESHOLD = 28.0;
 // Cập nhật lại giá trị theo cần thiết
 

float threshold1 = TEMP_THRESHOLD_1;
float threshold2 = TEMP_THRESHOLD_2;
float threshold3 = TEMP_THRESHOLD_3;
float threshold4 = TEMP_THRESHOLD_4;


// Bat dau giao tiep I2C
void i2c_start() {
gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
gpio_set_direction(I2C_SCL, GPIO_MODE_OUTPUT);
gpio_set_level(I2C_SDA, 0);
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
gpio_set_level(I2C_SCL, 0);
}

// Dung giao tiep I2C
void i2c_stop() {
gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
gpio_set_level(I2C_SDA, 0);
gpio_set_level(I2C_SCL, 1);
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
gpio_set_level(I2C_SDA, 1);
}

// Ghi byte qua I2C
void i2c_write_byte(unsigned char data) {
for (int i = 0; i < 8; i++) {
    gpio_set_level(I2C_SDA, (data & 0x80) ? 1 : 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
    gpio_set_level(I2C_SCL, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
    gpio_set_level(I2C_SCL, 0);
    data <<= 1;
}
gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT); // Nhan ACK
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
gpio_set_level(I2C_SCL, 1);
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(5)
gpio_set_level(I2C_SCL, 0);
gpio_set_direction(I2C_SDA, GPIO_MODE_OUTPUT);
}

// Gui lenh den LCD
void lcd_send_cmd(char cmd) {
char data_high = cmd & 0xF0;
char data_low = (cmd << 4) & 0xF0;

i2c_start();
i2c_write_byte(LCD_ADDR << 1);
i2c_write_byte(data_high | LCD_BACKLIGHT | LCD_ENABLE);
i2c_write_byte(data_high | LCD_BACKLIGHT);
i2c_write_byte(data_low | LCD_BACKLIGHT | LCD_ENABLE);
i2c_write_byte(data_low | LCD_BACKLIGHT);
i2c_stop();
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(500)
}

// Gui du lieu den LCD
void lcd_send_data(char data) {
char data_high = (data & 0xF0) | 0x01;
char data_low = ((data << 4) & 0xF0) | 0x01;

i2c_start();
i2c_write_byte(LCD_ADDR << 1);
i2c_write_byte(data_high | LCD_BACKLIGHT | LCD_ENABLE);
i2c_write_byte(data_high | LCD_BACKLIGHT);
i2c_write_byte(data_low | LCD_BACKLIGHT | LCD_ENABLE);
i2c_write_byte(data_low | LCD_BACKLIGHT);
i2c_stop();
vTaskDelay(1 / portTICK_PERIOD_MS);  // Thay the usleep(500)
}

// Khoi tao LCD
void lcd_init() {
lcd_send_cmd(0x33); // Khoi dong LCD 4-bit mode
lcd_send_cmd(0x32);
lcd_send_cmd(0x28); // Chi thi 2 dong, 5x8 font
lcd_send_cmd(0x0C); // Bat man hinh, tat con tro
lcd_send_cmd(0x06); // Chi thi nhap tu dong tang
lcd_send_cmd(0x01); // Xoa man hinh
vTaskDelay(5 / portTICK_PERIOD_MS);
}

// Dat con tro cho LCD
void lcd_set_cursor(int row, int col) {
int offsets[] = {0x80, 0xC0};
lcd_send_cmd(offsets[row] + col);
}

// In chuoi len LCD
void lcd_print(const char *str) {
while (*str) {
    lcd_send_data(*str++);
}
}
void lcd_clear() {
    lcd_send_cmd(0x01);  // Lệnh clear màn hình
    vTaskDelay(pdMS_TO_TICKS(2));  // Delay để LCD xử lý
}

void ds18b20_reset()
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_GPIO, 0);
    esp_rom_delay_us(480);
    gpio_set_level(DS18B20_GPIO, 1);
    esp_rom_delay_us(70);
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
    esp_rom_delay_us(410);
}

void ds18b20_write_bit(int bit)
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(DS18B20_GPIO, bit);
    esp_rom_delay_us(60);
    gpio_set_level(DS18B20_GPIO, 1);
    esp_rom_delay_us(2);
}

int ds18b20_read_bit()
{
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DS18B20_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level(DS18B20_GPIO, 1);
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
    esp_rom_delay_us(10);
    int bit = gpio_get_level(DS18B20_GPIO);
    esp_rom_delay_us(50);
    return bit;
}

void ds18b20_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        ds18b20_write_bit(data & 0x01);
        data >>= 1;
    }
}

uint8_t ds18b20_read_byte()
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++)
    {
        data |= (ds18b20_read_bit() << i);
    }
    return data;
}

float ds18b20_get_temperature()
{
    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM
    ds18b20_write_byte(0x44); // Start conversion
    vTaskDelay(pdMS_TO_TICKS(750)); // Chờ chuyển đổi nhiệt độ

    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM
    ds18b20_write_byte(0xBE); // Read Scratchpad

    uint8_t lsb = ds18b20_read_byte();
    uint8_t msb = ds18b20_read_byte();
    int16_t temp = (msb << 8) | lsb;

    return temp / 16.0;
}
int dht11_read(int *temperature, int *humidity)
{
uint8_t data[5] = {0};
gpio_set_direction(DHT11_GPIO, GPIO_MODE_OUTPUT);
gpio_set_level(DHT11_GPIO, 0);
vTaskDelay(pdMS_TO_TICKS(20));
gpio_set_level(DHT11_GPIO, 1);
esp_rom_delay_us(30);
gpio_set_direction(DHT11_GPIO, GPIO_MODE_INPUT);

int wait = 0;
// Chờ DHT11 phản hồi (LOW 80us + HIGH 80us)
while (gpio_get_level(DHT11_GPIO) == 1) { esp_rom_delay_us(1); if (++wait > 100) return -1; }
wait = 0;
while (gpio_get_level(DHT11_GPIO) == 0) { esp_rom_delay_us(1); if (++wait > 100) return -2; }
wait = 0;
while (gpio_get_level(DHT11_GPIO) == 1) { esp_rom_delay_us(1); if (++wait > 100) return -3; }

// Đọc 40 bit
for (int i = 0; i < 40; i++) {
    // Chờ start bit
    while (gpio_get_level(DHT11_GPIO) == 0);

    int time_high = 0;
    while (gpio_get_level(DHT11_GPIO) == 1) {
        time_high++;
        esp_rom_delay_us(1);
        if (time_high > 100) break;
    }

    data[i/8] <<= 1;
    if (time_high > 40) data[i/8] |= 1;
}

// Checksum
if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) != data[4]) return -4;

*humidity = data[0];
*temperature = data[2];
return 0;
}

bool is_relay_button_pressed()
{
static bool last_state = 1;  // Biến trạng thái nút trước đó (giả sử mức logic cao là 1)
bool current_state = gpio_get_level(BUTTON_RELAY_GPIO); // Đọc trạng thái nút hiện tại

// Kiểm tra nếu nút thay đổi từ trạng thái "nhả" sang "bấm"
if (last_state == 1 && current_state == 0)
{
    vTaskDelay(pdMS_TO_TICKS(50));  // Delay ngắn để chống rung (debounce)
    last_state = 0;  // Cập nhật trạng thái nút sau khi nhấn
    return true;  // Trả về true nếu nút được nhấn
}
if (current_state == 1)
{
    last_state = 1;  // Cập nhật lại trạng thái khi nút đã nhả
}
return false;
}

bool is_button_pressed()
{
if (gpio_get_level(BUTTON_MODE_GPIO) == 0)  // nút nhấn mức thấp
{
    vTaskDelay(pdMS_TO_TICKS(20)); // debounce nhẹ
    if (gpio_get_level(BUTTON_MODE_GPIO) == 0)
    {
        return true;  // chắc chắn nút đã nhấn
    }
}
return false;  // không nhấn
}

void init_gpio() {
    // Cấu hình GPIO cho relay
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 0); // Tắt rơ le ban đầu

    // Cấu hình nút MODE
    gpio_set_direction(BUTTON_MODE_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_MODE_GPIO);

    // Cấu hình nút nhấn điều khiển relay
    gpio_set_direction(BUTTON_RELAY_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_RELAY_GPIO);  // Kích hoạt pull-up cho nút nhấn

    // Cấu hình LED cảnh báo
    gpio_set_direction(ALERT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(ALERT_LED_GPIO, 0); // Tắt LED cảnh báo ban đầu

    // Khai báo io_conf một lần
    gpio_config_t io_conf;

    // Cấu hình relay GPIO (tạo ra lại io_conf)
    io_conf.pin_bit_mask = (1ULL << RELAY_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Cấu hình ALERT_LED_GPIO sử dụng lại io_conf
    io_conf.pin_bit_mask = (1ULL << ALERT_LED_GPIO);
    gpio_config(&io_conf);

    // Cấu hình BUTTON_UP và BUTTON_DOWN GPIO
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << BUTTON_UP_GPIO) | (1ULL << BUTTON_DOWN_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

// Hàm cập nhật LCD
void update_lcd(float ds18b20_temp, int dht11_temp, int dht11_humi)
{
char buf1[17];
char buf2[17];

// Dòng 0: Temp + Mode
lcd_set_cursor(0, 0);
lcd_print("                "); // Clear dòng
lcd_set_cursor(0, 0);
snprintf(buf1, sizeof(buf1), "T:%.1fC %s", ds18b20_temp, auto_mode ? "AUTO" : "MAN");
lcd_print(buf1);

// Dòng 1: Humidity
lcd_set_cursor(1, 0);
lcd_print("                "); // Clear dòng
lcd_set_cursor(1, 0);
snprintf(buf2, sizeof(buf2), "H:%d%%", dht11_humi);
lcd_print(buf2);
}

// Hàm kiểm tra nút UP (tăng ngưỡng)
bool is_button_up_pressed(void)
{
    if (gpio_get_level(BUTTON_UP_GPIO) == 0)  // Nút nhấn mức thấp
    {
        vTaskDelay(pdMS_TO_TICKS(20)); // Debounce nhẹ
        if (gpio_get_level(BUTTON_UP_GPIO) == 0)
        {
            return true;  // Chắc chắn nút đã nhấn
        }
    }
    return false;  // Không nhấn
}

// Hàm kiểm tra nút DOWN (giảm ngưỡng)
bool is_button_down_pressed(void)
{
    if (gpio_get_level(BUTTON_DOWN_GPIO) == 0)  // Nút nhấn mức thấp
    {
        vTaskDelay(pdMS_TO_TICKS(20)); // Debounce nhẹ
        if (gpio_get_level(BUTTON_DOWN_GPIO) == 0)
        {
            return true;  // Chắc chắn nút đã nhấn
        }
    }
    return false;  // Không nhấn
}
// Hàm thay đổi ngưỡng
void adjust_thresholds()
{
    bool changed = false;

    if (is_button_up_pressed())
    {
        switch (menu_state)
        {
            case 1: threshold1 += 0.5; changed = true; break;
            case 2: threshold2 += 0.5; changed = true; break;
            case 3: threshold3 += 0.5; changed = true; break;
            case 4: threshold4 += 0.5; changed = true; break;
        }
    }

    if (is_button_down_pressed())
    {
        switch (menu_state)
        {
            case 1: threshold1 -= 0.5; changed = true; break;
            case 2: threshold2 -= 0.5; changed = true; break;
            case 3: threshold3 -= 0.5; changed = true; break;
            case 4: threshold4 -= 0.5; changed = true; break;
        }
    }

    if (changed)
    {
        // Cập nhật lại LCD ngay sau khi điều chỉnh
        lcd_clear();
        char buf[32];
        switch (menu_state)
        {
            case 1:
                snprintf(buf, sizeof(buf), "T1: %.1fC", threshold1);
                break;
            case 2:
                snprintf(buf, sizeof(buf), "T2: %.1fC", threshold2);
                break;
            case 3:
                snprintf(buf, sizeof(buf), "T3: %.1fC", threshold3);
                break;
            case 4:
                snprintf(buf, sizeof(buf), "T4: %.1fC", threshold4);
                break;
        }
        lcd_set_cursor(0, 0);
        lcd_print(buf);

        vTaskDelay(pdMS_TO_TICKS(200)); // Debounce sau khi thay đổi
    }
}

// Hàm điều khiển relay theo ngưỡng
void control_relay(float ds18b20_temp)
{
    static bool relay_state = false;  // Biến lưu trạng thái relay

    // Cập nhật ngưỡng dựa trên nút bấm
    adjust_thresholds();

    if (auto_mode)
    {
        if (ds18b20_temp < threshold1 || ds18b20_temp > threshold2)
            gpio_set_level(RELAY_GPIO, 1);  // Bật rơ le
        else
            gpio_set_level(RELAY_GPIO, 0);  // Tắt rơ le
    }
    else
    {
        // MANUAL: Kiểm tra nút nhấn để điều khiển relay
        if (is_relay_button_pressed())
        {
            relay_state = !relay_state;  // Chuyển đổi trạng thái relay
            gpio_set_level(RELAY_GPIO, relay_state ? 1 : 0);
        }
    }
}

void check_temperature_alert(float temp)
{
    static int64_t last_toggle = 0;
    static bool led_state = false;
    int64_t now = esp_timer_get_time(); // microseconds

    if (temp < threshold3 || temp > threshold4)

    {
        if (now - last_toggle >= 500000) // 500ms
        {
            led_state = !led_state;
            gpio_set_level(ALERT_LED_GPIO, led_state);
            last_toggle = now;
        }
    }
    else
    {
        gpio_set_level(ALERT_LED_GPIO, 0); // tắt LED nếu không vượt ngưỡng
        led_state = false;
        last_toggle = now; // reset timer
    }
}


void update_threshold_and_mode() {
    switch (menu_state) {
        case 1:
            threshold1 = 24.0; // Ngưỡng 1
            ESP_LOGI("MENU", "1:Ngưỡng thấp = %.1f°C", threshold1);
            break;
        case 2:
             threshold2 = 28.0; // Ngưỡng 2
            ESP_LOGI("MENU", "2:Ngưỡng cao = %.1f°C", TEMP_HIGH_THRESHOLD);
            break;
        case 3:
            TEMP_LOW_THRESHOLD = 20.0; // Ngưỡng 3
            ESP_LOGI("MENU", "3:Ngưỡng thấp = %.1f°C", TEMP_LOW_THRESHOLD);
            break;
        case 4:
            TEMP_HIGH_THRESHOLD = 35.0; // Ngưỡng 4
            ESP_LOGI("MENU", "4:Ngưỡng cao = %.1f°C", TEMP_HIGH_THRESHOLD);
            break;
    }
}

bool is_menu_button_pressed() {
    return gpio_get_level(MENU_BUTTON_GPIO) == 0;  // 0 là trạng thái nhấn nút
}

void handle_menu() {
    static bool last_state = true;
    bool current = is_menu_button_pressed();
    int64_t now = esp_timer_get_time(); // microseconds

    if (current && last_state) {
        last_menu_interaction_time = now;

        if (!in_menu) {
            in_menu = true;
            menu_state = 1;
        } else {
            menu_state++;
            if (menu_state > 4) {
                menu_state = 0;
                in_menu = false;  // Thoát menu
                lcd_clear();
                return;
            }
        }

        //update_threshold_and_mode(); // Cập nhật giá trị ngưỡng

        // Hiển thị đơn giản từng ngưỡng riêng
        lcd_clear();
        char buf[32];
        switch (menu_state) {
            case 1:
                snprintf(buf, sizeof(buf), "T1: %.1fC", threshold1);
                break;
            case 2:
                snprintf(buf, sizeof(buf), "T2: %.1fC", threshold2);
                break;
            case 3:
                snprintf(buf, sizeof(buf), "T3: %.1fC", threshold3);
                break;
            case 4:
                snprintf(buf, sizeof(buf), "T4: %.1fC", threshold4);
                break;
        }
        lcd_set_cursor(0, 0);
        lcd_print(buf);

        vTaskDelay(pdMS_TO_TICKS(300)); // debounce
    }

    last_state = current;
}







void app_main() {
    lcd_init();
    init_gpio();
    float ds18b20_temp = 0.0;

    while (1) {
        handle_menu();  // xử lý MENU nhấn

        if (!in_menu) {
            // Đo và hiển thị cảm biến
            static uint32_t last_sensor_time = 0;
            if (esp_timer_get_time() / 1000 - last_sensor_time > 2000) {
                ds18b20_temp = ds18b20_get_temperature();
                int dht_temp = 0, dht_humi = 0;
                if (dht11_read(&dht_temp, &dht_humi) != 0) dht_humi = 0;
                update_lcd(ds18b20_temp, dht_temp, dht_humi);
                last_sensor_time = esp_timer_get_time() / 1000;
            }

            control_relay(ds18b20_temp);
            check_temperature_alert(ds18b20_temp);

            if (is_button_pressed()) {
                auto_mode = !auto_mode;
                vTaskDelay(pdMS_TO_TICKS(300));
            }
        }
        adjust_thresholds();

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


