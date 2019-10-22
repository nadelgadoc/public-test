import interpreter
import mqtt_sub
import irsend
import irget
import sht30
import utime
import gc
import ubinascii
import _thread
import json
import ntptime
import sys
import esp32
import urequests
import os
import dht
from machine import Timer
from machine import Pin
from machine import unique_id
from machine import reset
from machine import ADC
from machine import disable_irq,enable_irq
from machine import reset_cause
import playTone
import config
import custom_webserver
import test_helper

reset_reason_list = ['*','PWRON_RESET','HARD_RESET','WDT_RESET','DEEPSLEEP_RESET','SOFT_RESET']
net = locals()['net']

VERSION = 20191021
LED_BLUE = 4
LED_RED = 16
LED_GREEN = 17
BUZZER = 26
LED_IR = 5
SCL_PIN = 22
SDA_PIN = 23
ADC_PIN = 35
CURR_PIN = 33
DHT_PIN = 14

class main:
    def getAdcMinMax(self, adc_input):
        if not isinstance(adc_input, ADC):
            raise Exception('Input must be ADC Class')
        start_time = utime.ticks_ms()
        values = []
        while utime.ticks_diff(utime.ticks_ms(),start_time) < 100:
            values.append(adc_input.read())
            utime.sleep_us(200)
        return min(values),max(values)

    def getNormalizedRmsValue(self, min_value, max_value):
        zero = (min_value+max_value)/2.0
        vpeak = (max_value-zero)/4095.0
        return 0.707 * vpeak

    def raiseTimeout(self, dt):
        raise Exception('Task timedout!')

    def run(self):
        self.led_red.off()
        #self.irrcv._routine()
        msg = self.mqtt_client.getMsg()
        #if type(msg).__name__ == 'list':
        ##################
        if utime.ticks_ms() - self.prev_millis > self.uplink_period:
            json_msg = { "device": self.device_id, "temperature":0.0, "humidity": 0.0, "volt_sens":0.0, "curr_sens":0.0, "time":(ntptime.time() + 946684800)}
            print('Sending info')
            try:
                t_volt_converted = 0
                if not self.receiver_enabled:
                    volt_min,volt_max = self.getAdcMinMax(self.adc_input)
                    volt_rms = self.getNormalizedRmsValue(volt_min,volt_max)
                    t_volt_converted = volt_rms*3.3*self.vol_conversion
                if self.external_sensor_enabled:
                    try:
                        self.external_sensor.measure()
                        ext_temp = self.external_sensor.temperature()
                        ext_hum = self.external_sensor.humidity()
                        json_msg.update({'ext_temperature' : str(ext_temp)})
                        json_msg.update({'ext_humidity' : str(ext_hum)})
                    except Exception as e:
                        print('Exception reading external sensor')
                        sys.print_exception(e)
                else:
                    curr_read = self.curr_input.read()/4095.0*3.3
                    t_curr_converted = curr_read*self.curr_conversion
                if self.dht11_enabled or self.dht22_enabled:
                    try:
                        self.onboard_dht_sensor.measure()
                        dht_temp = self.onboard_dht_sensor.temperature()
                        dht_hum = self.onboard_dht_sensor.humidity()
                        json_msg.update({'dht_temperature' : str(dht_temp)})
                        json_msg.update({'dht_humidity' : str(dht_hum)})
                    except Exception as e:
                        print('Exception reading onboard dht sensor')
                        sys.print_exception(e)
                # sensor_data = self.sensor.measure(raw=True)
                # temperature = (sensor_data[0]*256)+sensor_data[1]
                # humidity = (sensor_data[3]*256)+sensor_data[4]
                temperature,humidity = self.sensor.measure()
                # core_temp = (esp32.raw_temperature() - 32) / 1.8
                # #temperature = (temperature-((8.5*int(enable_ap))+8.0+core_temp/22)) * self.temp_sensor_cal_prop + self.temp_sensor_cal_offset
                temperature = temperature * self.temp_sensor_cal_prop + self.temp_sensor_cal_offset
                humidity = humidity * self.hum_sensor_cal_prop + self.hum_sensor_cal_offset
                json_msg['temperature'] = str(temperature)
                json_msg['humidity'] = str(humidity)
                #json_msg['core_temp'] = str(core_temp)
                json_msg['volt_sens'] = str(t_volt_converted)
                #json_msg['curr_sens'] = str(t_curr_converted)
                self.mqtt_client.publishJson('esp32/uplink', json.dumps(json_msg))
                print(json_msg)
                print('OK')
            except Exception as e:
                print('Exception reading sensor: ' + str(e))
                sys.print_exception(e)
            self.prev_millis = utime.ticks_ms()
        utime.sleep(0.1)
        ####################
        if msg == {}:
            #print('Nothing to read')
            pass
        else:
            self.led_blue.off()
            #self.player.play(2489, 100)
            #interpreter.fillData(msg)
            if 'device' in msg:
                t_device = msg['device']
                print('Msg to ',t_device)
                if t_device != self.device_id:
                    print(t_device, ' != ', self.device_id)
                    self.led_red.on()
                    self.led_blue.on()
                    return
                else:
                    print(t_device, ' == ', self.device_id)
            self.player.play(2960, 200)
            t_verbose = False
            if 'verbose' in msg:
                t_verbose = True
            if 'version' in msg:
                self.msg_to_publish.update({ 'version' : VERSION })
            if 'last_version' in msg:
                t_last_version = msg['last_version']
                if t_last_version > VERSION:
                    self.msg_to_publish.update({ 'warning' : 'outdated' })
            if 'ping' in msg:
                t_ping = msg['ping']
                if t_ping == 'extra':
                    t_seconds_since_pwr_up = utime.time()
                    self.msg_to_publish.update({ 'seconds_since_power_up' : t_seconds_since_pwr_up })
                    self.msg_to_publish.update({ 'version' : VERSION })
                t_seconds_since_boot = utime.time() - self.boot_time
                t_epoch = ntptime.time()
                self.msg_to_publish.update({ 'seconds_since_boot' : t_seconds_since_boot })
                self.msg_to_publish.update({ 'epoch' : t_epoch })
            if 'wget' in msg:
                download_dict = msg['wget']
                save_to = ''
                if 'url' not in download_dict:
                    print('URL not specified')
                else:
                    url = download_dict['url']
                _url_splited = url.split('/')
                _url_splited = _url_splited[len(_url_splited)-1]
                if 'to' in download_dict:
                    save_to = download_dict['to']
                    if '.' not in save_to:
                        if save_to[len(save_to)] != '/':
                            save_to += '/'
                        save_to += _url_splited.split('?')[0]
                else:
                    print('Destination not specified')
                    save_to = _url_splited.split('?')[0]
                print('URL: ', url)
                print('Destination: ', save_to)
                dirs = save_to.split('/')
                for i in range(len(dirs)-1):
                    os.mkdir(dirs[i])
                try:
                    t_timer = Timer(1)
                    t_timer.init(period=10000, mode=Timer.ONE_SHOT, callback=self.raiseTimeout)
                    print('Downloading')
                    t_request = urequests.get(url)
                    print('Reading content text')
                    _content = t_request.text
                    print('Content OK')
                    if len(_content) < 20:
                        if '404' in _content:
                            raise Exception('Remote file not found')
                    print('Opening/Creating file')
                    _file = open(save_to, 'w')
                    print('Writting to file')
                    written_bytes = _file.write(_content)
                    print()
                    _file.close()
                    t_timer.deinit()
                    self.msg_to_publish.update({ 'donwload_file' : 'Done:' + str(written_bytes) })
                    self.player.play(2794, 200)
                    utime.sleep(0.1)
                    self.player.play(2794, 200)
                except Exception as e:
                    print('****Error downloading file')
                    sys.print_exception(e)
                    self.msg_to_publish.update({ 'download_file' : 'Failed' })
                    self.player.play(2489, 500)
                    utime.sleep(0.1)
            if 'listdir' in msg:
                t_dir = msg['listdir']
                if type(t_dir) == str:
                    dirs = os.listdir(t_dir)
                    self.msg_to_publish.update({ 'listdir' : { '/' + t_dir : dirs } })
                else:
                    dirs = os.listdir()
                    self.msg_to_publish.update({ 'listdir' : { '/' : dirs } })
            if 'get_secuence' in msg:
                t_secuence = 'sensor_disabled'
                if self.receiver_enabled:
                    print('Capturando')
                    t_secuence = self.irrcv.getOneSecuence()
                self.msg_to_publish.update({ 'secuence_detected' : t_secuence })
            if 'get_marked_secuence' in msg:
                t_marked_secuence = 'sensor_disabled'
                if self.receiver_enabled:
                    print('Capturando')
                    t_secuence = self.irrcv.getOneSecuence()
                    print('Convirtiendo')
                    self.data_handler.fillSecuence(t_secuence)
                    t_marked_secuence = self.data_handler.getMarkedSecuence()
                self.msg_to_publish.update({ "marked_secuence_detected": t_marked_secuence })
            if 'get_experimental_secuence' in msg:
                t_experimental_secuence = 'sensor_disabled'
                t_marks = msg['get_experimental_secuence']
                if type(t_marks) != list:
                    t_marks = [19000,8000,4000,1300]
                if self.receiver_enabled:
                    print('Capturando')
                    try:
                        t_experimental_secuence = self.helper.getDataAuto(t_marks[0],t_marks[1],t_marks[2],t_marks[3],verbose=t_verbose)
                    except Exception as e:
                        sys.print_exception(e)
                        t_experimental_secuence = 'wrong_marks'
                self.msg_to_publish.update({ 'experimental_secuence_detected': t_experimental_secuence })
            if 'get_settings' in msg:
                get_settings = msg['get_settings']
                t_get_config = {}
                if type(get_settings) == list:
                    print('Requesting config: ' + str(get_settings))
                    t_get_config = self.config_handler.getSettings(get_settings)
                elif type(get_settings) == str:
                    if get_settings == 'all':
                        print('Requesting all settings')
                        t_get_config = self.config_handler.getSettingsStruct()
                    else:
                        print('Requesting config: ' + get_settings)
                        t_get_config = self.config_handler.getSettings(get_settings)
                self.msg_to_publish.update({ 'local_settings':t_get_config })
            if 'settings' in msg:
                settings = msg['settings']
                if type(settings) == dict:
                    for varname in settings:
                        self.config_handler.saveSetting(varname, settings[varname])
                else:
                    print('Wrong json format')
            # else:
            #     print('No settings found in json')
            if 'frequency' in msg:
                freq = msg['frequency']
                self.irout.setFrequency(int(freq))
                print('Frequency fixed to: ' + str(freq))
            # else:
            #     print('No frequency found in json')
            if 'auto_calibration' in msg:
                cal_values = msg['auto_calibration']
                if type(cal_values) != dict:
                    print('Calibration values must be in a dict')
                else:
                    if 'temp' in cal_values:
                        target_temp = cal_values['temp']
                    elif 'temperature' in cal_values:
                        target_temp = cal_values['temperature']
                    else:
                        target_temp = None
                    if 'hum' in cal_values:
                        target_hum = cal_values['hum']
                    elif 'humidity' in cal_values:
                        target_hum = cal_values['humidity']
                    else:
                        target_hum = None
                    if type(target_temp) == float and type(target_hum) == float:
                        temperature,humidity = self.sensor.measure()
                        error_temp = target_temp - temperature
                        error_hum = target_hum - humidity
                        self.config_handler.saveSetting('sensor_calibration_temp', [1.0,error_temp])
                        self.config_handler.saveSetting('sensor_calibration_hum', [1.0,error_hum])
                        print('Calibration set to T: ' + str(error_temp) + 'C - H: ' + str(error_hum) + '%HR')
                    else:
                        print('Calibration values must be float')
            if 'secuence' in msg:
                secuence = msg['secuence']
                if type(secuence).__name__ != 'list':
                    print('Secuence wrong format')
                else:
                    print('Secuence OK')
                    if self.receiver_enabled:
                        t_irq = disable_irq()
                    len_sended = self.irout.sendSecuence(secuence)
                    if self.receiver_enabled:
                        enable_irq(t_irq)
                    print('Sended OK - ' + str(len_sended) + ' pulses')
            # else:
            #     print('No secuence detected in json')
            if 'reset' in msg:
                reset_delay = msg['reset']
                print('Reset request in ' + str(reset_delay) + 's')
                if reset_delay > 0:
                    utime.sleep(reset_delay)
                print('Rebooting')
                reset()
        if self.msg_to_publish != {}:
            self.msg_to_publish.update({'device':self.device_id})
            self.mqtt_client.publishJson('esp32/uplink', json.dumps(self.msg_to_publish))
            self.msg_to_publish = {}
            print('Publish OK')
            self.player.play(2489, 200)
        self.led_red.on()
        self.led_blue.on()
        gc.collect()
    
    def __init__(self):
        #interpreter = interpreter.interpreter()
        self.boot_time = utime.time()
        self.config_handler = config.local()
        self.receiver_enabled,self.external_sensor_enabled = self.config_handler.getSettings(['receiver_enabled','external_sensor_enabled'])
        if type(self.receiver_enabled) != bool:
            self.receiver_enabled = False
        if type(self.external_sensor_enabled) != bool:
            self.external_sensor_enabled = False
        # t_disable_local_config = self.config_handler.getSettings(['disable_local_config'])
        # if type(t_disable_local_config) != bool:
        #     t_disable_local_config = False
        # if t_disable_local_config:
        #     localserver.stop()
        self.dht11_enabled,self.dht22_enabled = self.config_handler.getSettings(['dht11_enabled','dht22_enabled'])
        if type(self.dht11_enabled) != bool:
            self.dht11_enabled = False
        if type(self.dht22_enabled) != bool:
            self.dht22_enabled = False
        self.data_handler = interpreter.interpreter()
        self.sensor = sht30.SHT30(scl_pin=SCL_PIN, sda_pin=SDA_PIN)
        if self.receiver_enabled:
            print('******Receiver enabled')
            self.irrcv = irget.receiver(ADC_PIN)
            self.helper = test_helper.helper(self.irrcv, self.data_handler)
        else:
            print('******Analog input enabled')
            self.adc_input = ADC(Pin(ADC_PIN))
            self.adc_input.atten(ADC.ATTN_11DB)
        if self.external_sensor_enabled:
            print('******External sensor enabled')
            self.external_sensor = dht.DHT22(Pin(CURR_PIN))
        else:
            self.curr_input = ADC(Pin(CURR_PIN))
            self.curr_input.atten(ADC.ATTN_11DB)
        if self.dht11_enabled:
            self.onboard_dht_sensor = dht.DHT11(Pin(DHT_PIN))
        elif self.dht22_enabled:
            self.onboard_dht_sensor = dht.DHT22(Pin(DHT_PIN))
        self.led_blue = Pin(LED_BLUE , Pin.OUT)
        self.led_red = Pin(LED_RED , Pin.OUT)
        self.led_green = Pin(LED_GREEN , Pin.OUT)
        self.led_red.on()   #leds turn on with ground, so on() turn it off
        self.led_green.on()
        self.led_blue.on()
        self.player = buzzer
        self.player.play(2217, 100)
        self.player.play(2489, 100)
        self.player.play(2794, 200)
        self.device_id = 'node-' + ubinascii.hexlify(unique_id()).decode('utf-8')
        self.irout = irsend.sender(38000, LED_IR)
        self.temp_sensor_cal_prop = 1.0
        self.temp_sensor_cal_offset = 0
        self.hum_sensor_cal_prop = 1.0
        self.hum_sensor_cal_offset = 0
        try:
            t_server, t_port, t_user, t_pass = self.config_handler.getSettings(['mqtt_server','mqtt_port','mqtt_user','mqtt_pass'])
        except:
            t_server = None
            t_port = None
            t_user = None
            t_pass = None
        self.mqtt_client = mqtt_sub.handler(devicename=self.device_id, server=t_server, port=t_port, username=t_user, password=t_pass)
        t_uplink_period, = self.config_handler.getSettings(['uplink_period'])
        if t_uplink_period == None:
            self.uplink_period = 300000
        else:
            self.uplink_period = (t_uplink_period if type(t_uplink_period) == int else int(t_uplink_period))
        self.curr_conversion, = self.config_handler.getSettings(['current_conversion'])
        if self.curr_conversion == None:
            self.curr_conversion = 1.0
        self.vol_conversion, = self.config_handler.getSettings(['vol_conversion'])
        if self.vol_conversion == None:
            self.vol_conversion = 1.0
        t_sensor_cal_tuple_temp, = self.config_handler.getSettings(['sensor_calibration_temp'])
        if type(t_sensor_cal_tuple_temp) == list:
            self.temp_sensor_cal_prop = t_sensor_cal_tuple_temp[0]
            self.temp_sensor_cal_offset = t_sensor_cal_tuple_temp[1]
        t_sensor_cal_tuple_hum, = self.config_handler.getSettings(['sensor_calibration_hum'])
        if type(t_sensor_cal_tuple_hum) == list:
            self.hum_sensor_cal_prop = t_sensor_cal_tuple_hum[0]
            self.hum_sensor_cal_offset = t_sensor_cal_tuple_hum[1]
        self.msg_to_publish = {}
        self.player.play(2489, 200)
        self.prev_millis = utime.ticks_ms()

enable_ap = False
reset_reason = reset_cause()
print('******Reset cause: ' + reset_reason_list[reset_reason])
if reset_reason != reset_reason_list.index('SOFT_RESET') or not net.isconnected():
    localserver = custom_webserver.server()
    enable_ap = True
local_boot_time = utime.time()
disconnected_time = utime.time()
buzzer = playTone.Player(BUZZER)
print('Boot successful')

while True:
    if 'script' not in locals():
        if net.isconnected():
            #ntptime.settime()
            print('Starting mqtt Client')
            script = main()
    elif script.mqtt_client.isConnected():
        # print('Mqtt Loop')
        script.led_green.on()
        try:
            script.mqtt_client.client.check_msg()
            utime.sleep(0.1)
            script.run()
        except OSError as e:
            print('OSError: ' + str(e))
            script.led_red.off()
            try:
                script.mqtt_client.disconnect()
            except:
                pass
        except Exception as e:
            print('Exception checking for msg: ' + str(e))
            sys.print_exception(e)
    else:
        try:
            script.mqtt_client.connect()
            script.led_red.on()
        except KeyboardInterrupt:
            sys.exit()
        except Exception as e:
            print('Exception connection to mqtt: ' + str(e))
            sys.print_exception(e)
            if net.isconnected() == False:
                print('No Internet connection')
                script.led_green.off()
            utime.sleep(1)
    # utime.sleep(0.5)
    # print('Main Loop')
    if not net.isconnected():
        if utime.time() - disconnected_time > 10:
            disconnected_time = utime.time()
            try:
                print('Scanning available networks')
                netlist = net.scan()
                print(netlist)
                try_to_connect = False
                for t_net in netlist:
                    if t_ssid.encode() in t_net:
                        try_to_connect = True
                if try_to_connect:
                    print('Attempting connection to \'' + str(t_ssid) + '\' with password \'' + str(t_pass) + '\'')
                    net.connect(t_ssid, t_pass)
                else:
                    print('Not Found')
            except Exception as e:
                print('****Exception: ' + str(e))
                print('Failed')
            if not net.isconnected():
                buzzer.play(2489, 400)
    if enable_ap:
        if utime.time() - local_boot_time > 300 and net.isconnected():
            enable_ap = False
            localserver.stop()
            print('Local server stopped')
        else:
            localserver.run()
    # print('Main Loop 2')
    # utime.sleep(0.5)